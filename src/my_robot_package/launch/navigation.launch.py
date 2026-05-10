import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import logging
# Set up logging to both console and file
log_dir = os.path.expanduser('./log')
os.makedirs(log_dir, exist_ok=True)
log_file = os.path.join(log_dir, 'navigation_launch.log')
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s %(levelname)s %(message)s',
    handlers=[
        logging.StreamHandler(),            # Console
        logging.FileHandler(log_file)       # File
    ]
)
logging.info("Starting navigation launch...")
def generate_launch_description():
    pkg_share = get_package_share_directory('my_robot_package')
    urdf_file = os.path.join(pkg_share, 'urdf', 'my_robot.urdf')
    lidar_launch_file = os.path.join(pkg_share, 'launch', 'ld19.launch.py')
    
    # POINT TO YOUR SAVED MAP HERE
    map_file = os.path.expanduser('/home/harsh/ros2_ws/src/my_robot_package/maps/my_house_map_0509_2.yaml')
    nav2_params_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')

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

        # 2. Hardware: Lidar
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
                'publish_tf': False   # <--- FIX: EKF handles TF now
            }]
        ),

        # 5. Hardware: IMU
        Node(
            package='my_robot_package',
            executable='imu_node',
            name='imu_node',
            output='screen'
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
            output='screen'
        ),

        # 6. Foxglove Bridge
        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            output='screen',
            parameters=[{'port': 8765}]
        ),

        # 7. NAV2 NODES (Explicitly launched)
        Node(
            package='nav2_controller',
            executable='controller_server',
            output='screen',
            parameters=[nav2_params_file]
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
            parameters=[nav2_params_file]
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