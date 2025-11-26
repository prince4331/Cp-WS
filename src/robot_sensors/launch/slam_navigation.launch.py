#!/usr/bin/env python3
"""
SLAM + Nav2 Navigation Launch File
Complete autonomous navigation system with mapping and localization
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Package directories
    robot_sensors_dir = get_package_share_directory('robot_sensors')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    
    # Config files
    slam_params_file = os.path.join(robot_sensors_dir, 'config', 'slam_params.yaml')
    nav2_params_file = os.path.join(robot_sensors_dir, 'config', 'nav2_params.yaml')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file_arg = LaunchConfiguration('slam_params_file')
    nav2_params_file_arg = LaunchConfiguration('nav2_params_file')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Use simulation (Gazebo) clock if true')
    
    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=slam_params_file,
        description='Full path to the ROS2 parameters file to use for SLAM Toolbox')
    
    declare_nav2_params_file_cmd = DeclareLaunchArgument(
        'nav2_params_file',
        default_value=nav2_params_file,
        description='Full path to the ROS2 parameters file to use for Nav2')
    
    # ===================================================================
    # 1. LIDAR Node (RPLIDAR A1M8)
    # ===================================================================
    lidar_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        parameters=[{
            'serial_port': '/dev/ttyUSB0',
            'serial_baudrate': 115200,
            'frame_id': 'laser',
            'inverted': False,
            'angle_compensate': True,
            'scan_mode': 'Standard'
        }],
        output='screen'
    )
    
    # ===================================================================
    # 2. Motor Controller
    # ===================================================================
    motor_controller_node = Node(
        package='robot_sensors',
        executable='motor_controller_node',
        name='motor_controller',
        output='screen'
    )
    
    # ===================================================================
    # 3. Multi-Sensor Bridge (Arduino communication)
    # ===================================================================
    multi_sensor_node = Node(
        package='robot_sensors',
        executable='multi_sensor_node',
        name='multi_sensor_bridge',
        parameters=[{
            'serial_port': '/dev/ttyACM0',
            'baud_rate': 115200
        }],
        output='screen'
    )
    
    # ===================================================================
    # 4. Odometry Publisher
    # ===================================================================
    odometry_node = Node(
        package='robot_sensors',
        executable='odometry_publisher',
        name='odometry_publisher',
        output='screen'
    )
    
    # ===================================================================
    # 5. Static Transform Publishers (Robot TF tree)
    # ===================================================================
    
    # base_link -> base_footprint (robot on ground)
    base_footprint_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_footprint_broadcaster',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint'],
        output='screen'
    )
    
    # base_link -> laser (LIDAR position)
    laser_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='laser_broadcaster',
        arguments=['0.1', '0', '0.15', '0', '0', '0', 'base_link', 'laser'],  # Adjust x, y, z, roll, pitch, yaw
        output='screen'
    )
    
    # ===================================================================
    # 6. SLAM Toolbox (Online Async SLAM)
    # ===================================================================
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params_file_arg, {'use_sim_time': use_sim_time}],
        remappings=[
            ('/scan', '/scan'),
            ('/tf', '/tf'),
            ('/tf_static', '/tf_static')
        ]
    )
    
    # ===================================================================
    # 7. Nav2 Navigation Stack
    # ===================================================================
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params_file_arg,
            'autostart': 'true',
        }.items()
    )
    
    # Delay Nav2 startup to allow SLAM to initialize
    delayed_nav2_bringup = TimerAction(
        period=5.0,
        actions=[nav2_bringup_launch]
    )
    
    # ===================================================================
    # Launch Description
    # ===================================================================
    ld = LaunchDescription()
    
    # Declare arguments
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_slam_params_file_cmd)
    ld.add_action(declare_nav2_params_file_cmd)
    
    # Add nodes in order
    ld.add_action(lidar_node)
    ld.add_action(motor_controller_node)
    ld.add_action(multi_sensor_node)
    ld.add_action(odometry_node)
    ld.add_action(base_footprint_tf)
    ld.add_action(laser_tf)
    ld.add_action(slam_toolbox_node)
    ld.add_action(delayed_nav2_bringup)  # Nav2 starts after 5 seconds
    
    return ld
