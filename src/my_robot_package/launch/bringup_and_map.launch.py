import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def validate_exploration_calibration(context):
    """Refuse autonomous frontier motion until critical dimensions are measured."""
    requested = LaunchConfiguration('enable_frontier_exploration').perform(context)
    if requested.strip().lower() not in {'true', '1', 'yes', 'on'}:
        return []

    package_share = get_package_share_directory('my_robot_package')
    status_path = os.path.join(
        package_share, 'config', 'calibration_status.yaml'
    )
    with open(status_path, 'r') as stream:
        status = yaml.safe_load(stream)
    required = (
        'wheel_geometry_measured',
        'wheel_speed_measured',
        'robot_footprint_measured',
        'imu_mount_and_bias_checked',
    )
    missing = [name for name in required if not status.get(name, False)]
    if missing:
        raise RuntimeError(
            'Frontier exploration is calibration-gated. Complete and record: '
            + ', '.join(missing)
        )
    return []


def generate_launch_description():
    pkg_share = get_package_share_directory('my_robot_package')
    urdf_file = os.path.join(pkg_share, 'urdf', 'my_robot.urdf')
    lidar_launch_file = os.path.join(pkg_share, 'launch', 'ld19.launch.py')

    slam_config = os.path.join(
        pkg_share,
        'config',
        'mapper_params_online_async.yaml'
    )
    enable_oakd_perception = LaunchConfiguration('enable_oakd_perception')
    enable_frontier_exploration = LaunchConfiguration(
        'enable_frontier_exploration'
    )
    enable_oakd_depth_obstacles = LaunchConfiguration(
        'enable_oakd_depth_obstacles'
    )
    start_oakd = PythonExpression([
        "'", enable_oakd_perception, "' == 'true' or '",
        enable_oakd_depth_obstacles, "' == 'true'",
    ])
    hardware_config = os.path.join(
        pkg_share, 'config', 'hardware_calibration.yaml'
    )
    twist_mux_config = os.path.join(pkg_share, 'config', 'twist_mux.yaml')
    nav2_params_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    explore_config = os.path.join(pkg_share, 'config', 'explore.yaml')
    tracking_config = os.path.join(
        pkg_share, 'config', 'person_tracking.yaml'
    )

    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        DeclareLaunchArgument(
            'enable_frontier_exploration',
            default_value='false',
            description=(
                'Use Nav2 frontier exploration instead of the legacy wall '
                'follower. Keep false until motion calibration is complete.'
            )
        ),
        DeclareLaunchArgument(
            'enable_oakd_depth_obstacles',
            default_value='false',
            description=(
                'Publish calibrated OAK-D depth for the optional frontier Nav2 '
                'costmaps. This does not replace LiDAR in SLAM Toolbox.'
            )
        ),
        DeclareLaunchArgument(
            'enable_oakd_perception',
            default_value='false',
            description=(
                'Start OAK-D RGB/person perception without allowing it to publish '
                'movement commands. LiDAR remains the SLAM sensor.'
            )
        ),
        LogInfo(
            msg='OAK-D perception enabled for mapping (camera does not drive).',
            condition=IfCondition(enable_oakd_perception)
        ),
        OpaqueFunction(function=validate_exploration_calibration),
        # 1. Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}]
        ),

        # 2. Hardware: Lidar (Included via launch file)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(lidar_launch_file)
        ),

        # 3. Hardware: Motor Driver
        Node(
            package='twist_mux',
            executable='twist_mux',
            name='twist_mux',
            output='screen',
            parameters=[twist_mux_config],
            remappings=[('/cmd_vel_out', '/cmd_vel_out')]
        ),
        Node(
            package='my_robot_package',
            executable='motor_driver_node',
            name='motor_driver_node',
            output='screen',
            parameters=[hardware_config]
        ),

        # 4. Hardware: Encoder Odom
        Node(
            package='my_robot_package',
            executable='encoder_odom_node',
            name='encoder_odom_node',
            output='screen',
            parameters=[hardware_config, {
                'base_frame': 'base_footprint',
                'odom_frame': 'odom',
                'publish_tf': False
            }]
        ),

        # 5. Hardware: IMU
        Node(
            package='my_robot_package',
            executable='imu_node',
            name='imu_node',
            output='screen',
            parameters=[hardware_config]
        ),

        # 5.5 Sensor Fusion (EKF)
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[os.path.join(pkg_share, 'config', 'ekf.yaml')]
        ),

        # 6. Foxglove Bridge
        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            output='screen',
            parameters=[{'port': 8765}]
        ),

        # Optional visual intelligence/recording. Motion stays disabled so the
        # selected mapping behavior remains the only autonomous motion source.
        Node(
            package='my_robot_package',
            executable='person_follower_node',
            name='oakd_perception_node',
            output='screen',
            parameters=[tracking_config, hardware_config, {
                'initial_mode': 'idle',
                'motion_enabled': False,
                'require_lidar': False,
                'publish_preview': True,
                'publish_pointcloud': ParameterValue(
                    enable_oakd_depth_obstacles, value_type=bool
                ),
            }],
            condition=IfCondition(start_oakd)
        ),

        # 7. SLAM Toolbox (Using Custom Config)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('slam_toolbox'),
                '/launch/online_async_launch.py'
            ]),
            launch_arguments={'slam_params_file': slam_config}.items()
        ),

        # Mapping behavior consumes these Range topics. The wall follower reads
        # bumpers directly. In frontier mode it is absent, so bumper_node becomes
        # the single GPIO owner and restores the direct motor emergency-stop path.
        Node(
            package='my_robot_package',
            executable='ultra_sensor_node',
            name='ultra_sensor_node',
            output='screen',
            parameters=[{'frame_id': 'ultrasonic_link'}]
        ),
        Node(
            package='my_robot_package',
            executable='ir_driver',
            name='ir_sensor_node',
            output='screen',
            parameters=[{
                'left_frame_id': 'ir_left_link',
                'right_frame_id': 'ir_right_link'
            }]
        ),
        Node(
            package='my_robot_package',
            executable='bumper_node',
            name='bumper_node',
            output='screen',
            parameters=[{'frame_id': 'base_footprint'}],
            condition=IfCondition(enable_frontier_exploration)
        ),

        # 8. Autonomous Wall Follower
        Node(
            package='my_robot_package',
            executable='wall_follower',
            name='wall_follower',
            output='screen',
            remappings=[('/cmd_vel', '/cmd_vel/mapping')],
            condition=UnlessCondition(enable_frontier_exploration)
        ),

        # Optional Nav2 stack used only by explore_lite while SLAM owns map->odom.
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[nav2_params_file],
            remappings=[('/cmd_vel', '/cmd_vel/navigation')],
            condition=IfCondition(enable_frontier_exploration)
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[nav2_params_file],
            condition=IfCondition(enable_frontier_exploration)
        ),
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[nav2_params_file],
            remappings=[('/cmd_vel', '/cmd_vel/navigation')],
            condition=IfCondition(enable_frontier_exploration)
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[nav2_params_file],
            condition=IfCondition(enable_frontier_exploration)
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_exploration',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'autostart': True,
                'node_names': [
                    'controller_server',
                    'planner_server',
                    'behavior_server',
                    'bt_navigator',
                ],
            }],
            condition=IfCondition(enable_frontier_exploration)
        ),
        Node(
            package='explore_lite',
            executable='explore',
            name='explore_node',
            output='screen',
            parameters=[explore_config],
            condition=IfCondition(enable_frontier_exploration)
        ),
    ])
