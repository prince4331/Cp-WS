from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    """
    Complete launch file for autonomous floor cleaning robot
    Industry-grade configuration with all sensors and navigation
    """
    
    # Declare launch arguments
    wheel_diameter_arg = DeclareLaunchArgument(
        'wheel_diameter', default_value='0.065',
        description='Wheel diameter in meters')
    
    wheel_base_arg = DeclareLaunchArgument(
        'wheel_base', default_value='0.25',
        description='Distance between wheels in meters')
    
    ticks_per_rev_arg = DeclareLaunchArgument(
        'ticks_per_revolution', default_value='800',
        description='Encoder ticks per wheel revolution')
    
    max_speed_arg = DeclareLaunchArgument(
        'max_speed', default_value='0.5',
        description='Maximum robot speed in m/s')
    
    arduino_port_arg = DeclareLaunchArgument(
        'arduino_port', default_value='/dev/ttyACM0',
        description='Arduino serial port')
    
    lidar_port_arg = DeclareLaunchArgument(
        'lidar_port', default_value='/dev/ttyUSB0',
        description='LIDAR serial port')
    
    return LaunchDescription([
        # Launch arguments
        wheel_diameter_arg,
        wheel_base_arg,
        ticks_per_rev_arg,
        max_speed_arg,
        arduino_port_arg,
        lidar_port_arg,
        
        # 1. LIDAR Node
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            parameters=[{
                'serial_port': LaunchConfiguration('lidar_port'),
                'serial_baudrate': 115200,
                'frame_id': 'laser_frame',
                'angle_compensate': True,
                'scan_mode': 'Sensitivity',
            }],
            output='screen'
        ),
        
        # 2. Multi-Sensor Bridge Node (Arduino)
        # Publishes: 5x Ultrasonic, 8x IR, 2x Encoder raw data
        Node(
            package='robot_sensors',
            executable='multi_sensor_node',
            name='multi_sensor_bridge',
            parameters=[{
                'port': LaunchConfiguration('arduino_port'),
                'baud': 115200,
            }],
            output='screen'
        ),
        
        # 3. Odometry Node
        # Converts encoder data to pose and velocity
        # Publishes: /odom, TF (odom→base_link)
        Node(
            package='robot_sensors',
            executable='odometry_node',
            name='odometry_node',
            parameters=[{
                'wheel_diameter': LaunchConfiguration('wheel_diameter'),
                'wheel_base': LaunchConfiguration('wheel_base'),
                'ticks_per_revolution': LaunchConfiguration('ticks_per_revolution'),
                'publish_tf': True,
            }],
            output='screen'
        ),
        
        # 4. Motor Controller Node
        # Subscribes: /cmd_vel
        # Controls: Motors via Arduino serial
        Node(
            package='robot_sensors',
            executable='motor_controller_node',
            name='motor_controller',
            parameters=[{
                'wheel_base': LaunchConfiguration('wheel_base'),
                'max_speed': LaunchConfiguration('max_speed'),
                'port': LaunchConfiguration('arduino_port'),
                'baud': 115200,
            }],
            output='screen'
        ),
        
        # 5. Static TF Publisher
        # Publishes: All sensor frame transforms
        Node(
            package='robot_sensors',
            executable='robot_tf_publisher',
            name='robot_tf_publisher',
            output='screen'
        ),
        
        # 6. Robot State Publisher (optional, for visualization)
        # Uncomment when URDF is ready
        # Node(
        #     package='robot_state_publisher',
        #     executable='robot_state_publisher',
        #     name='robot_state_publisher',
        #     output='screen'
        # ),
    ])
