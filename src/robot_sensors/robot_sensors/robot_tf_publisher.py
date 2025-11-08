#!/usr/bin/env python3
"""
Static TF Publisher for Floor Cleaning Robot
Publishes static transforms for all sensor frames
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster
import math

class RobotTFPublisher(Node):
    def __init__(self):
        super().__init__('robot_tf_publisher')
        
        self.tf_broadcaster = StaticTransformBroadcaster(self)
        
        # Publish all static transforms
        transforms = []
        
        # Base link to sensors
        # Ultrasonic sensors
        transforms.append(self.make_transform('base_link', 'ultrasonic_front', 0.15, 0.0, 0.1, 0))
        transforms.append(self.make_transform('base_link', 'ultrasonic_front_right', 0.12, -0.08, 0.1, -0.785))  # -45deg
        transforms.append(self.make_transform('base_link', 'ultrasonic_front_left', 0.12, 0.08, 0.1, 0.785))   # +45deg
        transforms.append(self.make_transform('base_link', 'ultrasonic_right', 0.0, -0.12, 0.1, -1.57))  # -90deg
        transforms.append(self.make_transform('base_link', 'ultrasonic_left', 0.0, 0.12, 0.1, 1.57))    # +90deg
        
        # LIDAR
        transforms.append(self.make_transform('base_link', 'laser_frame', 0.0, 0.0, 0.15, 0))
        
        # IR sensors (mounted low for floor/stair detection)
        transforms.append(self.make_transform('base_link', 'ir_front_right', 0.12, -0.06, 0.03, 0))
        transforms.append(self.make_transform('base_link', 'ir_front_left', 0.12, 0.06, 0.03, 0))
        transforms.append(self.make_transform('base_link', 'ir_back_right', -0.12, -0.06, 0.03, 3.14))
        transforms.append(self.make_transform('base_link', 'ir_back_left', -0.12, 0.06, 0.03, 3.14))
        
        # Water level sensors (mounted in tanks)
        transforms.append(self.make_transform('base_link', 'water_clean_level', -0.10, 0.08, 0.25, 0))
        transforms.append(self.make_transform('base_link', 'water_dirty_level', -0.10, -0.08, 0.25, 0))

        # IMU
        transforms.append(self.make_transform('base_link', 'imu_link', 0.0, 0.0, 0.12, 0))

        # Base footprint (projection on ground)
        transforms.append(self.make_transform('base_link', 'base_footprint', 0.0, 0.0, -0.05, 0))
        
        # Publish all transforms
        self.tf_broadcaster.sendTransform(transforms)
        
        self.get_logger().info(f'Published {len(transforms)} static transforms')
    
    def make_transform(self, parent, child, x, y, z, yaw):
        """Create a transform"""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = parent
        t.child_frame_id = child
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z
        
        # Convert yaw to quaternion
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = math.sin(yaw / 2.0)
        t.transform.rotation.w = math.cos(yaw / 2.0)
        
        return t

def main(args=None):
    rclpy.init(args=args)
    node = RobotTFPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
