import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('my_robot_package')
    urdf_file = os.path.join(pkg_share, 'urdf', 'my_robot.urdf')
    lidar_launch_file = os.path.join(pkg_share, 'launch', 'ld19.launch.py')

    # --- THIS VARIABLE MUST BE DEFINED HERE (BEFORE THE LIST STARTS) ---
    slam_config = os.path.join(
        pkg_share,
        'config',
        'mapper_params_online_async.yaml'
    )
    # -------------------------------------------------------------------

    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
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
            package='my_robot_package',
            executable='motor_driver_node',
            name='motor_driver_node',
            output='screen'
        ),

        # 4. Hardware: Encoder Odom
        Node(
            package='my_robot_package',
            executable='encoder_odom_node',
            name='encoder_odom_node',
            output='screen',
            parameters=[{
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
            output='screen'
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

        # 7. SLAM Toolbox (Using Custom Config)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('slam_toolbox'),
                '/launch/online_async_launch.py'
            ]),
            launch_arguments={'slam_params_file': slam_config}.items()
        ), # <--- Notice the comma added here

        # # 8. Nav2 Navigation Stack (The Driver)
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([
        #         get_package_share_directory('nav2_bringup'),
        #         '/launch/navigation_launch.py'
        #     ]),
        #     launch_arguments={
        #         'use_sim_time': 'false',
        #         'params_file': os.path.join(get_package_share_directory('my_robot_package'), 'config', 'mapper_nav2_params.yaml') # <--- ONLY used during mapping
        #     }.items()
        # ),


        # 8. Autonomous Frontier Exploration
        # TimerAction(
        #     period=15.0,
        #     actions=[
        #         Node(
        #             package='explore_lite',
        #             executable='explore',
        #             name='explore_node',
        #             output='screen',
        #             parameters=[{
        #                 'robot_base_frame': 'base_footprint',
        #                 'costmap_topic': '/map',           # <--- CHANGE THIS
        #                 'costmap_updates_topic': '/map_updates', # <--- CHANGE THIS
        #                 'visualize': True,
        #                 'planner_frequency': 0.33,
        #                 'progress_timeout': 60.0,
        #                 'potential_scale': 3.0,
        #                 'orientation_scale': 0.0,
        #                 'gain_scale': 1.0,
        #                 'transform_tolerance': 0.3,
        #                 'min_frontier_size': 0.25,
        #             }]
        #         )
        #     ]
        # ),

        # Node(
        #     package='my_robot_package',
        #     executable='bumper_node',
        #     name='bumper_node',
        #     output='screen'
        # ),



        # 8. Autonomous Wall Follower
        Node(
            package='my_robot_package',
            executable='wall_follower',
            name='wall_follower',
            output='screen'
        ),
    ])