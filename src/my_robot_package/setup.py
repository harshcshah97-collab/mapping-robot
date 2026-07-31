from setuptools import setup
import os
from glob import glob

package_name = 'my_robot_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # INCLUDE THESE LINES:
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='My Autonomous Robot',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_driver_node = my_robot_package.motor_driver_node:main',
            'encoder_odom_node = my_robot_package.encoder_odom_node:main',
            'ultra_sensor_node = my_robot_package.ultra_sensor_node:main',
            'imu_node = my_robot_package.imu_node:main',
            'ir_driver = my_robot_package.ir_driver:main',
            'random_walk = my_robot_package.random_walk:main',
            'wall_follower = my_robot_package.wall_follower:main',
            'bumper_node = my_robot_package.bumper_node:main',
            'assistant_node = my_robot_package.assistant_node:main',
            'oakd_tracker_node = my_robot_package.oakd_tracker_node:main',
            'person_follower_node = my_robot_package.person_follower_node:main',
                        # REMOVE slam_tf_bridge_node if present
        ],
    },
)
