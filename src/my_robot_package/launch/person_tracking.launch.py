import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    my_robot_package_dir = get_package_share_directory('my_robot_package')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # --- Use your existing navigation launch file ---
    # This assumes you have a launch file that starts your base drivers and URDF.
    # If not, you'll need to add those nodes here.
    robot_base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(my_robot_package_dir, 'launch', 'navigation.launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'map': os.path.join(my_robot_package_dir, 'maps', 'my_map.yaml') # Or your desired map
        }.items()
    )

    # --- Person Follower Node ---
    person_follower_node = Node(
        package='my_robot_package',
        executable='person_follower_node',
        name='person_follower_node',
        output='screen',
        # Remap if your camera frame is different
        remappings=[
            ('/goal_pose', '/goal_pose')
        ]
    )

    # --- Assistant Node ---
    assistant_node = Node(
        package='my_robot_package',
        executable='assistant_node',
        name='assistant_node',
        output='screen'
    )

    return LaunchDescription([
        LogInfo(msg="Starting robot base drivers and Nav2 stack..."),
        robot_base_launch,
        
        LogInfo(msg="Starting person follower node..."),
        person_follower_node,
        
        LogInfo(msg="Starting assistant node..."),
        assistant_node
    ])
