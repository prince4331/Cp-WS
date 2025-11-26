#!/usr/bin/env python3
"""
Launch file for Autonomous Floor Cleaning Robot (Odometry-based)
Uses odometry + IMU instead of AMCL for localization
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    map_arg = DeclareLaunchArgument(
        'map',
        default_value='',
        description='Path to map YAML file (e.g., /home/saad/clean_ws/my_first_map.yaml)'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    # Get parameter values
    map_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Map Server Node (for coverage planning only, not for localization)
    # Using lifecycle manager with bond timeout disabled for reliability
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': map_file,
            'use_sim_time': use_sim_time,
            'bond_disable_heartbeat_timeout': True
        }]
    )
    
    # Lifecycle Manager for Map Server with extended timeout
    map_lifecycle_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='map_lifecycle_manager',
        output='screen',
        parameters=[{
            'autostart': True,
            'node_names': ['map_server'],
            'bond_timeout': 10.0,
            'attempt_respawn_reconnection': True,
            'bond_respawn_max_duration': 10.0
        }]
    )
    
    # Coverage Path Planner Node
    coverage_planner_node = Node(
        package='robot_sensors',
        executable='coverage_path_planner',
        name='coverage_path_planner',
        output='screen',
        parameters=[{
            'stripe_width': 0.4,  # 40cm cleaning width
            'safety_margin': 0.3,  # 30cm from obstacles
            'min_area_size': 0.5,  # minimum 0.5m² area
        }]
    )
    
    # Cleaning Controller Node (using odometry)
    cleaning_controller_node = Node(
        package='robot_sensors',
        executable='cleaning_controller',
        name='cleaning_controller',
        output='screen',
        parameters=[{
            'max_linear_speed': 0.25,  # Slower for odometry-only
            'max_angular_speed': 0.4,
            'waypoint_tolerance': 0.3,  # Larger tolerance for odometry
            'lookahead_distance': 0.5,
        }]
    )
    
    # Safety Monitor Node
    safety_monitor_node = Node(
        package='robot_sensors',
        executable='safety_monitor',
        name='safety_monitor',
        output='screen',
        parameters=[{
            'obstacle_distance_threshold': 0.25,  # 25cm emergency stop
            'warning_distance_threshold': 0.40,   # 40cm warning
            'stair_threshold': 400,  # IR sensor threshold
            'check_front_arc': 90.0,  # Check 90° front arc
        }]
    )
    
    # Buzzer Controller Node
    buzzer_controller_node = Node(
        package='robot_sensors',
        executable='buzzer_controller',
        name='buzzer_controller',
        output='screen'
    )
    
    return LaunchDescription([
        map_arg,
        use_sim_time_arg,
        LogInfo(msg=['Starting Autonomous Cleaning Robot (Odometry Mode) with map: ', map_file]),
        map_server_node,
        map_lifecycle_node,
        coverage_planner_node,
        cleaning_controller_node,
        safety_monitor_node,
        buzzer_controller_node,
    ])
