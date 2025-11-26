#!/usr/bin/env python3
"""
Simple Map Publisher - bypasses lifecycle complexity
Directly publishes the map for coverage planning
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
import yaml
from PIL import Image
import numpy as np
import sys


class SimpleMapPublisher(Node):
    def __init__(self, map_yaml_file):
        super().__init__('simple_map_publisher')
        
        # Create publisher with TRANSIENT_LOCAL for late joiners
        map_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.publisher = self.create_publisher(OccupancyGrid, '/map', map_qos)
        
        # Load and publish map
        self.load_and_publish_map(map_yaml_file)
        
        # Keep publishing periodically for reliability
        self.timer = self.create_timer(5.0, self.publish_map)
        
        self.get_logger().info(f'Simple Map Publisher started with map: {map_yaml_file}')
    
    def load_and_publish_map(self, yaml_file):
        """Load map from YAML and PGM files"""
        try:
            # Load YAML
            with open(yaml_file, 'r') as f:
                map_data = yaml.safe_load(f)
            
            # Get map parameters
            resolution = map_data['resolution']
            origin = map_data['origin']
            
            # Load PGM image
            image_file = yaml_file.replace('.yaml', '.pgm')
            img = Image.open(image_file).convert('L')  # Convert to grayscale
            img_array = np.array(img)
            
            # Create OccupancyGrid message
            self.map_msg = OccupancyGrid()
            self.map_msg.header.frame_id = 'map'
            self.map_msg.header.stamp = self.get_clock().now().to_msg()
            
            self.map_msg.info.resolution = resolution
            self.map_msg.info.width = img_array.shape[1]
            self.map_msg.info.height = img_array.shape[0]
            self.map_msg.info.origin.position.x = origin[0]
            self.map_msg.info.origin.position.y = origin[1]
            self.map_msg.info.origin.position.z = 0.0
            self.map_msg.info.origin.orientation.w = 1.0
            
            # Convert image to occupancy data
            # PGM: 254=free, 0=occupied, 205=unknown
            occupancy = np.zeros(img_array.shape, dtype=np.int8)
            occupancy[img_array == 254] = 0    # Free
            occupancy[img_array == 0] = 100    # Occupied
            occupancy[img_array == 205] = -1   # Unknown
            
            # Flatten and convert to list
            self.map_msg.data = occupancy.flatten().tolist()
            
            # Publish immediately
            self.publisher.publish(self.map_msg)
            self.get_logger().info(f'Map published! Size: {self.map_msg.info.width}x{self.map_msg.info.height}, Resolution: {resolution}m')
            
        except Exception as e:
            self.get_logger().error(f'Failed to load map: {e}')
            sys.exit(1)
    
    def publish_map(self):
        """Republish map periodically"""
        self.map_msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(self.map_msg)


def main(args=None):
    if len(sys.argv) < 2:
        print('Usage: simple_map_publisher.py <map.yaml>')
        sys.exit(1)
    
    rclpy.init(args=args)
    node = SimpleMapPublisher(sys.argv[1])
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
