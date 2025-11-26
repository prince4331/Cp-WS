#!/usr/bin/env python3
"""
Launch file for Autonomous Floor Cleaning Robot
Starts all necessary nodes for coverage-based cleaning operation
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


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
    
    # Map Server Node
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': map_file,
            'use_sim_time': use_sim_time
        }]
    )
    
    # Lifecycle Manager for Map Server
    map_lifecycle_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='map_lifecycle_manager',
        output='screen',
        parameters=[{
            'autostart': True,
            'node_names': ['map_server']
        }]
    )
    
    # AMCL Localization Node
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'alpha1': 0.2,
            'alpha2': 0.2,
            'alpha3': 0.2,
            'alpha4': 0.2,
            'alpha5': 0.2,
            'base_frame_id': 'base_link',
            'beam_skip_distance': 0.5,
            'beam_skip_error_threshold': 0.9,
            'beam_skip_threshold': 0.3,
            'do_beamskip': False,
            'global_frame_id': 'map',
            'lambda_short': 0.1,
            'laser_likelihood_max_dist': 2.0,
            'laser_max_range': 12.0,
            'laser_min_range': 0.15,
            'laser_model_type': 'likelihood_field',
            'max_beams': 60,
            'max_particles': 2000,
            'min_particles': 500,
            'odom_frame_id': 'odom',
            'pf_err': 0.05,
            'pf_z': 0.99,
            'recovery_alpha_fast': 0.0,
            'recovery_alpha_slow': 0.0,
            'resample_interval': 1,
            'robot_model_type': 'differential',
            'save_pose_rate': 0.5,
            'sigma_hit': 0.2,
            'tf_broadcast': True,
            'transform_tolerance': 1.0,
            'update_min_a': 0.2,
            'update_min_d': 0.25,
            'z_hit': 0.5,
            'z_max': 0.05,
            'z_rand': 0.5,
            'z_short': 0.05,
            'scan_topic': 'scan',
            'set_initial_pose': False,
        }]
    )
    
    # Lifecycle Manager for AMCL
    amcl_lifecycle_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='amcl_lifecycle_manager',
        output='screen',
        parameters=[{
            'autostart': True,
            'node_names': ['amcl']
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
    
    # Cleaning Controller Node
    cleaning_controller_node = Node(
        package='robot_sensors',
        executable='cleaning_controller',
        name='cleaning_controller',
        output='screen',
        parameters=[{
            'max_linear_speed': 0.3,  # Slow speed for cleaning
            'max_angular_speed': 0.5,
            'waypoint_tolerance': 0.2,
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
        LogInfo(msg=['Starting Autonomous Cleaning Robot with map: ', map_file]),
        map_server_node,
        map_lifecycle_node,
        amcl_node,
        amcl_lifecycle_node,
        coverage_planner_node,
        cleaning_controller_node,
        safety_monitor_node,
        buzzer_controller_node,
    ])
