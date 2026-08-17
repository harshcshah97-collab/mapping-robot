from setuptools import setup
import os
from glob import glob

package_name = 'my_robot_package'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
        (os.path.join('share', package_name, 'web_ui'), glob('web_ui/*')),
    ],
    install_requires=[
        'setuptools',
        'openai',
        'SpeechRecognition',
        'ddgs',
        'Flask',
        'bless',
    ],
    zip_safe=True,
    maintainer='Harsh Shah',
    maintainer_email='harshcshah97@gmail.com',
    description='ROS 2 companion robot mapping, navigation, and visual tracking',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_driver_node = my_robot_package.motor_driver_node:main',
            'encoder_odom_node = my_robot_package.encoder_odom_node:main',
            'ultra_sensor_node = my_robot_package.ultra_sensor_node:main',
            'imu_node = my_robot_package.imu_node:main',
            'ir_driver = my_robot_package.ir_driver:main',
            'wall_follower = my_robot_package.wall_follower:main',
            'bumper_node = my_robot_package.bumper_node:main',
            'assistant_node = my_robot_package.assistant_node:main',
            'person_follower_node = my_robot_package.person_follower_node:main',
            'check_vision = my_robot_package.check_vision:main',
            'oakd_ai_test = my_robot_package.oakd_ai_test:main',
            'calibration_check = my_robot_package.calibration_check:main',
            'perception_benchmark = my_robot_package.perception_benchmark:main',
        ],
    },
)
