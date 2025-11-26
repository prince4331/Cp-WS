#!/usr/bin/env python3
"""
Fixed RPLIDAR A1 M8 Launch File
Problem: Default launch file doesn't pass scan_mode parameter correctly
Solution: Direct node execution with explicit parameters
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            output='screen',
            parameters=[{
                'channel_type': 'serial',
                'serial_port': '/dev/ttyUSB0',
                'serial_baudrate': 115200,
                'scan_mode': 'Standard',  # Critical: Must be explicitly set
                'frame_id': 'laser',
                'inverted': False,
                'angle_compensate': True,
            }]
        ),
    ])
