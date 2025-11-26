#!/usr/bin/env python3
"""
Launch file for Advanced Autonomous Cleaner
Complete multi-sensor autonomous cleaning system
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Advanced Autonomous Cleaner (main navigation)
        Node(
            package='robot_sensors',
            executable='advanced_autonomous_cleaner',
            name='advanced_autonomous_cleaner',
            output='screen',
            parameters=[{
                'use_sim_time': False,
            }]
        ),
    ])
