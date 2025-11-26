#!/usr/bin/env python3
"""
Enhanced Reactive Navigation Launch File
Launches LIDAR + Sensors + Motors + Enhanced Cleaner + Dashboard
"""

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. RPLIDAR - 360° laser scanner
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            parameters=[{
                'serial_port': '/dev/ttyUSB0',
                'frame_id': 'laser',
                'angle_compensate': True,
                'scan_mode': 'Standard'
            }],
            output='screen'
        ),
        
        # 2. Arduino Multi-Sensor Bridge (5 Ultrasonic + 8 IR + Encoders)
        Node(
            package='robot_sensors',
            executable='multi_sensor_node',
            name='multi_sensor_node',
            output='screen'
        ),
        
        # 3. Motor Controller (cmd_vel -> PWM)
        Node(
            package='robot_sensors',
            executable='motor_controller_node',
            name='motor_controller_node',
            output='screen'
        ),
        
        # 4. Odometry Publisher (Encoders + IMU fusion)
        Node(
            package='robot_sensors',
            executable='odometry_publisher',
            name='odometry_publisher',
            output='screen'
        ),
        
        # 5. TF: base_footprint -> base_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_footprint_to_base_link',
            arguments=['0', '0', '0.05', '0', '0', '0', 'base_footprint', 'base_link']
        ),
        
        # 6. TF: base_link -> laser
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_laser',
            arguments=['0.15', '0', '0.12', '0', '0', '0', 'base_link', 'laser']
        ),
        
        # 7. Enhanced Smart Cleaner - Best reactive navigation
        Node(
            package='robot_sensors',
            executable='enhanced_smart_cleaner',
            name='enhanced_smart_cleaner',
            output='screen'
        ),
        
        # 8. Enhanced Dashboard - Map visualization
        Node(
            package='robot_sensors',
            executable='enhanced_dashboard',
            name='enhanced_dashboard',
            output='screen'
        ),
    ])
