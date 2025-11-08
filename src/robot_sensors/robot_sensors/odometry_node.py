#!/usr/bin/env python3
"""
Differential Drive Odometry Node
Calculates robot pose from wheel encoder data
Publishes odometry messages and TF transforms
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
from std_msgs.msg import Int32
from tf2_ros import TransformBroadcaster
import math

class DiffDriveOdometry(Node):
    def __init__(self):
        super().__init__('odometry_node')
        
        # Robot physical parameters
        self.declare_parameter('wheel_diameter', 0.065)  # meters (65mm wheels)
        self.declare_parameter('wheel_base', 0.25)       # meters (distance between wheels)
        self.declare_parameter('ticks_per_revolution', 800)  # encoder ticks per wheel revolution
        self.declare_parameter('publish_tf', True)
        
        # Get parameters
        wheel_dia = self.get_parameter('wheel_diameter').value
        self.wheel_base = self.get_parameter('wheel_base').value
        ticks_per_rev = self.get_parameter('ticks_per_revolution').value
        self.publish_tf = self.get_parameter('publish_tf').value
        
        # Calculate meters per tick
        wheel_circumference = math.pi * wheel_dia
        self.meters_per_tick = wheel_circumference / ticks_per_rev
        
        # Odometry state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.vx = 0.0
        self.vth = 0.0
        
        # Previous encoder values
        self.last_left_ticks = None
        self.last_right_ticks = None
        self.last_time = self.get_clock().now()
        
        # Subscribers
        self.sub_left = self.create_subscription(
            Int32, '/encoder/left', self.left_encoder_callback, 10)
        self.sub_right = self.create_subscription(
            Int32, '/encoder/right', self.right_encoder_callback, 10)
        
        # Publisher
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        
        # TF broadcaster
        if self.publish_tf:
            self.tf_broadcaster = TransformBroadcaster(self)
        
        # Storage for latest encoder values
        self.left_ticks = 0
        self.right_ticks = 0
        self.left_updated = False
        self.right_updated = False
        
        # Timer to compute odometry
        self.create_timer(0.05, self.compute_odometry)  # 20 Hz
        
        self.get_logger().info('Differential drive odometry node started')
        self.get_logger().info(f'Wheel diameter: {wheel_dia}m, Base: {self.wheel_base}m')
        self.get_logger().info(f'Meters per tick: {self.meters_per_tick:.6f}m')
    
    def left_encoder_callback(self, msg):
        self.left_ticks = msg.data
        self.left_updated = True
    
    def right_encoder_callback(self, msg):
        self.right_ticks = msg.data
        self.right_updated = True
    
    def compute_odometry(self):
        # Wait until we have data from both encoders
        if not (self.left_updated and self.right_updated):
            return
        
        current_time = self.get_clock().now()
        
        # Initialize on first run
        if self.last_left_ticks is None:
            self.last_left_ticks = self.left_ticks
            self.last_right_ticks = self.right_ticks
            self.last_time = current_time
            return
        
        # Calculate tick differences
        delta_left = self.left_ticks - self.last_left_ticks
        delta_right = self.right_ticks - self.last_right_ticks
        
        # Calculate distance traveled by each wheel
        left_dist = delta_left * self.meters_per_tick
        right_dist = delta_right * self.meters_per_tick
        
        # Calculate robot motion
        delta_dist = (left_dist + right_dist) / 2.0
        delta_theta = (right_dist - left_dist) / self.wheel_base
        
        # Update pose
        if abs(delta_theta) < 1e-6:
            # Moving straight
            self.x += delta_dist * math.cos(self.theta)
            self.y += delta_dist * math.sin(self.theta)
        else:
            # Arc motion
            radius = delta_dist / delta_theta
            self.x += radius * (math.sin(self.theta + delta_theta) - math.sin(self.theta))
            self.y -= radius * (math.cos(self.theta + delta_theta) - math.cos(self.theta))
        
        self.theta += delta_theta
        
        # Normalize theta to [-pi, pi]
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        
        # Calculate velocities
        dt = (current_time - self.last_time).nanoseconds / 1e9
        if dt > 0:
            self.vx = delta_dist / dt
            self.vth = delta_theta / dt
        
        # Publish odometry
        self.publish_odometry(current_time)
        
        # Update last values
        self.last_left_ticks = self.left_ticks
        self.last_right_ticks = self.right_ticks
        self.last_time = current_time
    
    def publish_odometry(self, current_time):
        # Create odometry message
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        
        # Set position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        
        # Set orientation (convert theta to quaternion)
        odom.pose.pose.orientation = self.euler_to_quaternion(0, 0, self.theta)
        
        # Set velocity
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = self.vth
        
        # Set covariance (rough estimates)
        odom.pose.covariance[0] = 0.01   # x
        odom.pose.covariance[7] = 0.01   # y
        odom.pose.covariance[35] = 0.05  # theta
        odom.twist.covariance[0] = 0.01  # vx
        odom.twist.covariance[35] = 0.05 # vth
        
        # Publish
        self.odom_pub.publish(odom)
        
        # Publish TF transform
        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = current_time.to_msg()
            t.header.frame_id = 'odom'
            t.child_frame_id = 'base_link'
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0
            t.transform.rotation = self.euler_to_quaternion(0, 0, self.theta)
            self.tf_broadcaster.sendTransform(t)
    
    def euler_to_quaternion(self, roll, pitch, yaw):
        """Convert Euler angles to quaternion"""
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        
        q = Quaternion()
        q.x = qx
        q.y = qy
        q.z = qz
        q.w = qw
        return q

def main(args=None):
    rclpy.init(args=args)
    node = DiffDriveOdometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
