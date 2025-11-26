#!/usr/bin/env python3
"""
Full system launch file
Starts Arduino multi-sensor bridge, LiDAR, motor controller,
autonomous navigation node, rosbridge, TF tree, and SLAM toolbox.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import (
    PythonLaunchDescriptionSource,
    XMLLaunchDescriptionSource,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    lidar_port = LaunchConfiguration('lidar_port')
    arduino_port = LaunchConfiguration('arduino_port')

    declare_lidar = DeclareLaunchArgument(
        'lidar_port',
        default_value='/dev/ttyUSB0',
        description='Serial port for the RPLIDAR',
    )

    declare_arduino = DeclareLaunchArgument(
        'arduino_port',
        default_value='/dev/ttyACM0',
        description='Serial port for the Arduino multi-sensor bridge',
    )

    multi_sensor_node = Node(
        package='robot_sensors',
        executable='multi_sensor_node',
        name='multi_sensor_bridge',
        output='screen',
        parameters=[{'port': arduino_port}],
    )

    motor_controller_node = Node(
        package='robot_sensors',
        executable='motor_controller_node',
        name='motor_controller',
        output='screen',
        parameters=[{'port': arduino_port}],
    )

    industry_avoidance_node = Node(
        package='robot_sensors',
        executable='industry_obstacle_avoidance',
        name='industry_obstacle_avoidance',
        output='screen',
    )

    sllidar_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        output='screen',
        parameters=[{
            'channel_type': 'serial',
            'serial_port': lidar_port,
            'serial_baudrate': 115200,
            'frame_id': 'laser',
            'angle_compensate': True,
        }],
    )

    tf_odom_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_odom_base',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_footprint'],
    )

    tf_base_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_link',
        arguments=['0', '0', '0.05', '0', '0', '0', 'base_footprint', 'base_link'],
    )

    tf_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_laser',
        arguments=['0.15', '0', '0.12', '0', '0', '0', 'base_link', 'laser'],
    )

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('slam_toolbox'),
                'launch',
                'online_async_launch.py',
            )
        )
    )

    rosbridge_launch = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('rosbridge_server'),
                'launch',
                'rosbridge_websocket_launch.xml',
            )
        )
    )

    return LaunchDescription([
        declare_lidar,
        declare_arduino,
        multi_sensor_node,
        motor_controller_node,
        industry_avoidance_node,
        sllidar_node,
        tf_odom_base,
        tf_base_link,
        tf_laser,
        slam_launch,
        rosbridge_launch,
    ])
