from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'robot_sensors'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*.sh') if os.path.exists('scripts') else []),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='saad',
    maintainer_email='saad@todo.todo',
    description='Multi-sensor ROS 2 bridge for autonomous floor cleaning robot: 5x Ultrasonic, 8x IR, 2x Encoder sensors',
    license='MIT',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            # Single ultrasonic sensor node (legacy)
            'serial_range_node = robot_sensors.serial_range_node:main',
            # Multi-sensor node (5x ultrasonic, 8x IR, 2x encoder)
            'multi_sensor_node = robot_sensors.multi_sensor_node:main',
            # Odometry node (converts encoder data to pose)
            'odometry_node = robot_sensors.odometry_node:main',
            # Motor controller (cmd_vel to motor commands)
            'motor_controller_node = robot_sensors.motor_controller_node:main',
            # TF publisher (static transforms for all sensors)
            'robot_tf_publisher = robot_sensors.robot_tf_publisher:main',
            # Real-time sensor dashboard
            'sensor_dashboard = robot_sensors.sensor_dashboard:main',
            # Web-based dashboard
            'web_dashboard = robot_sensors.web_dashboard:main',
        ],
    },
)
