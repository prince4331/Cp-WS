from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # LIDAR Node
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            parameters=[{
                'serial_port': '/dev/ttyUSB0',
                'serial_baudrate': 115200,
                'frame_id': 'laser_frame',
            }],
            output='screen'
        ),
        
        # Ultrasonic Sensor Node
        Node(
            package='robot_sensors',
            executable='serial_range_node',
            name='ultrasonic_sensor',
            parameters=[{
                'port': '/dev/ttyACM0',
                'baud': 115200,
                'topic': '/ultrasonic/range',
                'frame_id': 'ultrasonic_front',
            }],
            output='screen'
        ),
    ])
