#!/usr/bin/env python3
"""
Motor Controller Node
Subscribes to /cmd_vel and sends motor commands to Arduino
Implements differential drive kinematics and emergency stop handling
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool


class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')
        self._declare_parameters()
        self._setup_interfaces()
        self._init_state()
        self.get_logger().info('Motor controller node started')
        self.get_logger().info(f'Wheel base: {self.wheel_base}m, Max speed: {self.max_speed}m/s')

    def _declare_parameters(self):
        # Robot parameters
        self.declare_parameter('wheel_base', 0.25)  # meters
        self.declare_parameter('max_speed', 1.5)    # m/s
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)

        self.wheel_base = self.get_parameter('wheel_base').value
        self.max_speed = self.get_parameter('max_speed').value

    def _setup_interfaces(self):
        # Subscribe to velocity commands
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        # Publisher for motor PWM commands (to multi_sensor_node)
        self.motor_pwm_pub = self.create_publisher(String, '/motor/pwm', 10)

        # Emergency / safety subscriptions
        self.estop_active = False
        self.estop_sub = self.create_subscription(Bool, '/emergency_stop', self.emergency_stop_cb, 10)
        self.safety_estop_sub = self.create_subscription(Bool, '/safety/estop_main', self.emergency_stop_cb, 10)
        self.obstacle_emergency_sub = self.create_subscription(Bool, '/obstacle/emergency', self.obstacle_emergency_cb, 10)

        # Publisher for motor status
        self.status_pub = self.create_publisher(String, '/motor/status', 10)

    def _init_state(self):
        # Safety timeout
        self.last_cmd_time = self.get_clock().now()
        self.timeout = 0.5  # seconds
        self.create_timer(0.1, self.check_timeout)

        # Track last sent PWM values to avoid duplicate commands
        self.last_left_pwm = 0
        self.last_right_pwm = 0
        self.stopped = True

    def cmd_vel_callback(self, msg: Twist):
        """Convert Twist message to motor commands"""
        # If E-STOP is active, ignore velocity commands
        if self.estop_active:
            self.get_logger().warn('E-STOP active: ignoring cmd_vel')
            # still update last_cmd_time to prevent timeout logic from firing
            self.last_cmd_time = self.get_clock().now()
            return

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

        self.get_logger().info(
            f'cmd_vel: lin={linear_vel:.2f} ang={angular_vel:.2f} '
            f'-> PWM L={pwm_left} R={pwm_right}'
        )

    def velocity_to_pwm(self, velocity: float) -> int:
        if abs(velocity) < 0.01:
            return 0

        MIN_PWM = 160
        MAX_PWM = 255

        velocity_ratio = abs(velocity) / self.max_speed
        pwm_range = MAX_PWM - MIN_PWM
        pwm = MIN_PWM + int(velocity_ratio * pwm_range)

        if velocity < 0:
            pwm = -pwm

        return max(-MAX_PWM, min(MAX_PWM, pwm))

    def send_motor_command(self, left_pwm: int, right_pwm: int):
        if left_pwm == self.last_left_pwm and right_pwm == self.last_right_pwm:
            return

        pwm_msg = String()
        pwm_msg.data = f"{left_pwm},{right_pwm}"
        self.motor_pwm_pub.publish(pwm_msg)

        status_msg = String()
        status_msg.data = f"L:{left_pwm} R:{right_pwm}"
        self.status_pub.publish(status_msg)

        self.last_left_pwm = left_pwm
        self.last_right_pwm = right_pwm
        self.stopped = (left_pwm == 0 and right_pwm == 0)

    def check_timeout(self):
        time_since_cmd = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        if time_since_cmd > self.timeout and not self.stopped:
            self.send_motor_command(0, 0)

    def emergency_stop_cb(self, msg: Bool):
        if msg.data:
            if not self.estop_active:
                self.get_logger().warn('Emergency stop ENGAGED (dashboard/safety)')
            self.estop_active = True
            self.send_motor_command(0, 0)
        else:
            if self.estop_active:
                self.get_logger().info('Emergency stop RELEASED (dashboard/safety)')
            self.estop_active = False

    def obstacle_emergency_cb(self, msg: Bool):
        if msg.data:
            self.get_logger().warn('Obstacle Emergency: stopping motors')
            self.estop_active = True
            self.send_motor_command(0, 0)
        else:
            self.get_logger().info('Obstacle emergency cleared (manual re-enable required)')


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
