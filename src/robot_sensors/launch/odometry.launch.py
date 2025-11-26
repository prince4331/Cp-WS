#!/usr/bin/env python3
"""
Launch file for robot odometry system with EKF fusion
Starts: multi_sensor_node, odometry_publisher, robot_localization EKF
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('robot_sensors')
    ekf_config_file = os.path.join(pkg_dir, 'config', 'ekf.yaml')
    
    # Declare launch arguments
    wheel_radius_arg = DeclareLaunchArgument(
        'wheel_radius',
        default_value='0.1',
        description='Wheel radius in meters (measured: 0.1m = 10cm)'
    )
    
    wheel_base_arg = DeclareLaunchArgument(
        'wheel_base',
        default_value='0.255',
        description='Distance between wheels in meters (measured: 0.255m = 25.5cm)'
    )
    
    ticks_per_rev_arg = DeclareLaunchArgument(
        'ticks_per_rev',
        default_value='30',
        description='Encoder ticks per wheel revolution (measured: 30 ticks)'
    )
    
    use_imu_heading_arg = DeclareLaunchArgument(
        'use_imu_heading',
        default_value='true',
        description='Use IMU yaw for heading instead of encoder differential'
    )
    
    # Multi-sensor bridge node (Arduino interface)
    multi_sensor_node = Node(
        package='robot_sensors',
        executable='multi_sensor_node',
        name='multi_sensor_bridge',
        output='screen',
        parameters=[{
            'port': '/dev/ttyACM0',
            'baud': 115200,
            'encoder_left_invert': True,
            'encoder_right_invert': True,
        }]
    )
    
    # Odometry publisher node (encoder + IMU fusion)
    odometry_publisher_node = Node(
        package='robot_sensors',
        executable='odometry_publisher',
        name='odometry_publisher',
        output='screen',
        parameters=[{
            'wheel_radius': LaunchConfiguration('wheel_radius'),
            'wheel_base': LaunchConfiguration('wheel_base'),
            'encoder_ticks_per_rev': LaunchConfiguration('ticks_per_rev'),
            'use_imu_heading': LaunchConfiguration('use_imu_heading'),
            'magnetic_declination': -0.0066,  # -0.38° for Dhaka, Bangladesh (23.748°N, 90.425°E)
            'publish_tf': True,
            'odom_frame': 'odom',
            'base_frame': 'base_link',
        }]
    )
    
    # Robot localization EKF node (sensor fusion)
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_file],
        remappings=[
            ('/odometry/filtered', '/odom/filtered')
        ]
    )
    
    # Static transform: base_link to imu_link
    static_tf_base_to_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_imu_broadcaster',
        arguments=['0', '0', '0.05', '0', '0', '0', 'base_link', 'imu_link']
    )
    
    return LaunchDescription([
        wheel_radius_arg,
        wheel_base_arg,
        ticks_per_rev_arg,
        use_imu_heading_arg,
        multi_sensor_node,
        odometry_publisher_node,
        ekf_node,
        static_tf_base_to_imu,
    ])
