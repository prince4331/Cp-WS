#!/usr/bin/env python3
"""
Simple Robust Cleaner Launch File
==================================

Launches the dead-simple autonomous cleaner that ALWAYS works
No complex states, just effective cleaning
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        # Log level argument
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='Logging level (debug, info, warn, error)'
        ),
        
        # Simple Robust Cleaner Node
        Node(
            package='robot_sensors',
            executable='simple_robust_cleaner',
            name='simple_robust_cleaner',
            output='screen',
            emulate_tty=True,
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
            parameters=[{
                'use_sim_time': False,
            }]
        ),
    ])
