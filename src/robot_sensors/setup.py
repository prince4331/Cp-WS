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
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*.sh') if os.path.exists('scripts') else []),
        (os.path.join('share', package_name, 'templates'), glob('templates/*.html')),
    ],
    install_requires=['setuptools', 'flask', 'flask-cors'],
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
            # Odometry publisher with IMU fusion
            'odometry_publisher = robot_sensors.odometry_publisher:main',
            # Motor controller (cmd_vel to motor commands)
            'motor_controller_node = robot_sensors.motor_controller_node:main',
            # TF publisher (static transforms for all sensors)
            'robot_tf_publisher = robot_sensors.robot_tf_publisher:main',
            # Real-time sensor dashboard
            'sensor_dashboard = robot_sensors.sensor_dashboard:main',
            # Web-based dashboard
            'web_dashboard = robot_sensors.web_dashboard:main',
            # Autonomous navigation - obstacle avoidance
            'obstacle_avoidance_node = robot_sensors.obstacle_avoidance_node:main',
            # Floor cleaning robot nodes
            'coverage_path_planner = robot_sensors.coverage_path_planner:main',
            'cleaning_controller = robot_sensors.cleaning_controller:main',
            'safety_monitor = robot_sensors.safety_monitor:main',
            # Autonomous cleaner - reactive navigation (no SLAM)
            'autonomous_cleaner_node = robot_sensors.autonomous_cleaner_node:main',
            # Smart cleaner - sensor fusion (LIDAR + Ultrasonic + IR)
            'smart_cleaner_node = robot_sensors.smart_cleaner_node:main',
            # Enhanced smart cleaner - BEST multi-sensor reactive navigation
            'enhanced_smart_cleaner = robot_sensors.enhanced_smart_cleaner:main',
            # Professional cleaner - ROBUST multi-sensor with diagnostics
            'professional_cleaner = robot_sensors.professional_cleaner:main',
            # ADVANCED AUTONOMOUS CLEANER - Complete multi-sensor fusion (BEST)
            'advanced_autonomous_cleaner = robot_sensors.advanced_autonomous_cleaner:main',
            # Control dashboard - web interface
            'control_dashboard = robot_sensors.control_dashboard:main',
            # Enhanced dashboard - map visualization with real-time robot tracking
            'enhanced_dashboard = robot_sensors.enhanced_dashboard:main',
            'buzzer_controller = robot_sensors.buzzer_controller:main',
            # Simple map publisher (bypasses lifecycle complexity)
            'simple_map_publisher = robot_sensors.simple_map_publisher:main',
            # Wall-following cleaner - simple autonomous without path planning
            'wall_follow_cleaner = robot_sensors.wall_follow_cleaner:main',
            # SIMPLE ROBUST CLEANER - Dead-simple algorithm that ALWAYS WORKS
            'simple_robust_cleaner = robot_sensors.simple_robust_cleaner:main',
            # INDUSTRY-GRADE OBSTACLE AVOIDANCE - Multi-sensor fusion (LiDAR + 6 US + 8 IR)
            'industry_obstacle_avoidance = robot_sensors.industry_obstacle_avoidance:main',
        ],
    },
)
