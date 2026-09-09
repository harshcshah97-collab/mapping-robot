import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('my_robot_package')
    urdf_file = os.path.join(pkg_share, 'urdf', 'my_robot.urdf')
    lidar_launch_file = os.path.join(pkg_share, 'launch', 'ld19.launch.py')

    default_map_file = os.path.join(
        pkg_share, 'maps', 'my_house_map_0515.yaml'
    )
    map_file = LaunchConfiguration('map')
    nav2_params_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    hardware_config = os.path.join(
        pkg_share, 'config', 'hardware_calibration.yaml'
    )
    twist_mux_config = os.path.join(pkg_share, 'config', 'twist_mux.yaml')
    tracking_config = os.path.join(
        pkg_share, 'config', 'person_tracking.yaml'
    )
    enable_lidar = LaunchConfiguration('enable_lidar')
    enable_oakd_perception = LaunchConfiguration('enable_oakd_perception')
    enable_oakd_depth_obstacles = LaunchConfiguration(
        'enable_oakd_depth_obstacles'
    )
    start_oakd = PythonExpression([
        "'", enable_oakd_perception, "' == 'true' or '",
        enable_oakd_depth_obstacles, "' == 'true'",
    ])

    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=default_map_file,
            description='Absolute path to the saved map YAML file.'
        ),
        DeclareLaunchArgument(
            'enable_lidar',
            default_value='true',
            description=(
                'Start the LD19 LiDAR. Set false only when another launch file '
                'already owns the sensor.'
            )
        ),
        DeclareLaunchArgument(
            'enable_oakd_depth_obstacles',
            default_value='false',
            description=(
                'Publish calibrated OAK-D depth into Nav2 costmaps. The camera '
                'node refuses this unless hardware calibration marks the mount '
                'transform as confirmed.'
            )
        ),
        DeclareLaunchArgument(
            'enable_oakd_perception',
            default_value='false',
            description=(
                'Start OAK-D RGB/person perception without allowing it to publish '
                'movement commands. LiDAR remains the Nav2 obstacle sensor.'
            )
        ),
        LogInfo(
            msg='OAK-D perception enabled for navigation (camera does not drive).',
            condition=IfCondition(enable_oakd_perception)
        ),
        LogInfo(
            msg='OAK-D depth obstacles requested for Nav2.',
            condition=IfCondition(enable_oakd_depth_obstacles)
        ),
        # 1. Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}]
        ),

        # 2. Hardware: LiDAR
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(lidar_launch_file),
            condition=IfCondition(enable_lidar)
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

        # 5a. Sensor Fusion (EKF)
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[os.path.join(pkg_share, 'config', 'ekf.yaml')]
        ),

        # 5b. Hardware: Ultrasonic Sensor
        Node(
            package='my_robot_package',
            executable='ultra_sensor_node',
            name='ultra_sensor_node',
            output='screen',
            parameters=[{
                'frame_id': 'ultrasonic_link'
            }]
        ),

        # 5c. Hardware: IR Sensors
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

        # 5d. Hardware: Bumpers
        Node(
            package='my_robot_package',
            executable='bumper_node',
            name='bumper_node',
            output='screen',
            parameters=[{
                'frame_id': 'base_footprint'
            }]
        ),

        # 6. Foxglove Bridge
        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            output='screen',
            parameters=[{'port': 8765}]
        ),

        # 6a. Optional camera intelligence/recording layer. It never publishes
        # cmd_vel here, so it cannot fight Nav2 for control of the motors.
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

        # 7. NAV2 NODES (Explicitly launched)
        Node(
            package='nav2_controller',
            executable='controller_server',
            output='screen',
            parameters=[nav2_params_file],
            remappings=[('/cmd_vel', '/cmd_vel/navigation')]
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[nav2_params_file]
        ),
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[nav2_params_file],
            remappings=[('/cmd_vel', '/cmd_vel/navigation')]
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[nav2_params_file]
        ),
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[nav2_params_file, {'yaml_filename': map_file}]
        ),
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[nav2_params_file]
        ),

        # 8. LIFECYCLE MANAGER (The Conductor)
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{'use_sim_time': False},
                        {'autostart': True},
                        {'node_names': ['map_server',
                                        'amcl',
                                        'controller_server',
                                        'planner_server',
                                        'behavior_server',
                                        'bt_navigator']}]
        ),
    ])
