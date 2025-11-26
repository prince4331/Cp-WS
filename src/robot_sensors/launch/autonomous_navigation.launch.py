#!/usr/bin/env python3
"""
Launch file for testing autonomous obstacle avoidance
Starts all sensors + motor controller + obstacle avoidance
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Multi-sensor node (ultrasonic, IR, encoders)
        Node(
            package='robot_sensors',
            executable='multi_sensor_node',
            name='multi_sensor_bridge',
            output='screen',
            parameters=[{
                'serial_port': '/dev/ttyACM0',
                'baud_rate': 115200,
            }]
        ),
        
        # Motor controller (cmd_vel -> motor commands)
        Node(
            package='robot_sensors',
            executable='motor_controller_node',
            name='motor_controller',
            output='screen',
            parameters=[{
                'wheel_base': 0.25,
                'max_speed': 0.5,
            }]
        ),
        
        # Obstacle avoidance node (autonomous navigation)
        Node(
            package='robot_sensors',
            executable='obstacle_avoidance_node',
            name='obstacle_avoidance',
            output='screen',
        ),
        
        # LiDAR (optional, for additional safety)
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            parameters=[{
                'serial_port': '/dev/ttyUSB0',
                'frame_id': 'laser_frame',
                'angle_compensate': True,
                'scan_mode': 'Sensitivity',
            }],
            output='screen',
        ),
    ])
