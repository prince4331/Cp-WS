from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    Launch file for autonomous floor cleaning robot
    Starts: LIDAR, Multi-Sensor Bridge (5x Ultrasonic, 8x IR, 2x Encoder)
    """
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
        
        # Multi-Sensor Bridge Node (Arduino Mega)
        # Publishes:
        #   - 5x Ultrasonic Range sensors
        #   - 4x IR Object detection sensors
        #   - 4x IR Stair detection sensors
        #   - 2x Wheel encoders
        Node(
            package='robot_sensors',
            executable='multi_sensor_node',
            name='multi_sensor_bridge',
            parameters=[{
                'port': '/dev/ttyACM0',
                'baud': 115200,
            }],
            output='screen'
        ),
    ])
