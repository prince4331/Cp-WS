#!/usr/bin/env python3
"""
Motor Controller Node
Subscribes to /cmd_vel and sends motor commands to Arduino
Implements differential drive kinematics
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import math


class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')

        # Robot parameters
        self.declare_parameter('wheel_base', 0.25)  # meters
        self.declare_parameter('max_speed', 0.5)    # m/s
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)

        self.wheel_base = self.get_parameter('wheel_base').value
        self.max_speed = self.get_parameter('max_speed').value

        # Subscribe to velocity commands
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        # Publisher for motor PWM commands (to multi_sensor_node)
        self.motor_pwm_pub = self.create_publisher(String, '/motor/pwm', 10)
        
        # Publisher for motor status
        self.status_pub = self.create_publisher(String, '/motor/status', 10)

        # Safety timeout
        self.last_cmd_time = self.get_clock().now()
        self.timeout = 0.5  # seconds
        self.create_timer(0.1, self.check_timeout)

        self.get_logger().info('Motor controller node started')
        self.get_logger().info(f'Wheel base: {self.wheel_base}m, Max speed: {self.max_speed}m/s')

    def cmd_vel_callback(self, msg):
        """Convert Twist message to motor commands"""
        self.last_cmd_time = self.get_clock().now()

        # Get linear and angular velocities
        linear_vel = msg.linear.x
        angular_vel = msg.angular.z

        # Limit velocities
        linear_vel = max(-self.max_speed, min(self.max_speed, linear_vel))

        # Differential drive kinematics
        v_left = linear_vel - (angular_vel * self.wheel_base / 2.0)
        v_right = linear_vel + (angular_vel * self.wheel_base / 2.0)

        # Convert to PWM values (-255 to 255)
        pwm_left = self.velocity_to_pwm(v_left)
        pwm_right = self.velocity_to_pwm(v_right)

        # Send motor commands
        self.send_motor_command(pwm_left, pwm_right)

        self.get_logger().debug(
            f'cmd_vel: lin={linear_vel:.2f} ang={angular_vel:.2f} '
            f'-> PWM L={pwm_left} R={pwm_right}'
        )

    def velocity_to_pwm(self, velocity):
        pwm = int((velocity / self.max_speed) * 255)
        return max(-255, min(255, pwm))

    def send_motor_command(self, left_pwm, right_pwm):
        # Send PWM commands to multi_sensor_node
        pwm_msg = String()
        pwm_msg.data = f"{left_pwm},{right_pwm}"
        self.motor_pwm_pub.publish(pwm_msg)
        
        # Publish status
        status_msg = String()
        status_msg.data = f"L:{left_pwm} R:{right_pwm}"
        self.status_pub.publish(status_msg)

    def check_timeout(self):
        time_since_cmd = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        if time_since_cmd > self.timeout:
            self.send_motor_command(0, 0)


def main(args=None):
    rclpy.init(args=args)
    node = MotorController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.send_motor_command(0, 0)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
