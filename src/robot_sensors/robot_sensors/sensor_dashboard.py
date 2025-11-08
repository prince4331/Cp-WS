#!/usr/bin/env python3
"""
Real-time Sensor Dashboard for Floor Cleaning Robot
Displays all sensor data in a clean terminal interface
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range, LaserScan
from std_msgs.msg import Int32, String, Bool
from nav_msgs.msg import Odometry
import curses
import math
from datetime import datetime

class SensorDashboard(Node):
    def __init__(self):
        super().__init__('sensor_dashboard')
        
        # Sensor data storage
        self.ultrasonic_data = {
            'front': None,
            'front_left': None,
            'front_right': None,
            'left': None,
            'right': None
        }
        
        self.ir_object_data = {
            'front': None,
            'front_left': None,
            'front_right': None,
            'back': None
        }
        
        self.ir_stair_data = {
            'front': None,
            'front_left': None,
            'front_right': None,
            'back': None
        }
        
        self.encoder_data = {
            'left': 0,
            'right': 0
        }
        
        self.odom_data = {
            'x': 0.0,
            'y': 0.0,
            'theta': 0.0,
            'linear_vel': 0.0,
            'angular_vel': 0.0
        }
        
        self.lidar_data = {
            'range_count': 0,
            'min_range': 0.0,
            'max_range': 0.0,
            'scan_time': 0.0
        }
        
        self.raw_sensor_data = ""
        self.last_update = datetime.now()
        
        # Subscribe to all topics
        # Ultrasonic sensors
        self.create_subscription(Range, '/ultrasonic/front', 
                                lambda msg: self.update_ultrasonic('front', msg), 10)
        self.create_subscription(Range, '/ultrasonic/front_left', 
                                lambda msg: self.update_ultrasonic('front_left', msg), 10)
        self.create_subscription(Range, '/ultrasonic/front_right', 
                                lambda msg: self.update_ultrasonic('front_right', msg), 10)
        self.create_subscription(Range, '/ultrasonic/left', 
                                lambda msg: self.update_ultrasonic('left', msg), 10)
        self.create_subscription(Range, '/ultrasonic/right', 
                                lambda msg: self.update_ultrasonic('right', msg), 10)
        
        # IR Object Detection sensors
        self.create_subscription(Bool, '/ir_object/front', 
                                lambda msg: self.update_ir_object('front', msg), 10)
        self.create_subscription(Bool, '/ir_object/front_left', 
                                lambda msg: self.update_ir_object('front_left', msg), 10)
        self.create_subscription(Bool, '/ir_object/front_right', 
                                lambda msg: self.update_ir_object('front_right', msg), 10)
        self.create_subscription(Bool, '/ir_object/back', 
                                lambda msg: self.update_ir_object('back', msg), 10)
        
        # IR Stair Detection sensors
        self.create_subscription(Bool, '/ir_stair/front', 
                                lambda msg: self.update_ir_stair('front', msg), 10)
        self.create_subscription(Bool, '/ir_stair/front_left', 
                                lambda msg: self.update_ir_stair('front_left', msg), 10)
        self.create_subscription(Bool, '/ir_stair/front_right', 
                                lambda msg: self.update_ir_stair('front_right', msg), 10)
        self.create_subscription(Bool, '/ir_stair/back', 
                                lambda msg: self.update_ir_stair('back', msg), 10)
        
        # Encoders
        self.create_subscription(Int32, '/encoder/left', 
                                lambda msg: self.update_encoder('left', msg), 10)
        self.create_subscription(Int32, '/encoder/right', 
                                lambda msg: self.update_encoder('right', msg), 10)
        
        # Odometry
        self.create_subscription(Odometry, '/odom', self.update_odom, 10)
        
        # LIDAR
        self.create_subscription(LaserScan, '/scan', self.update_lidar, 10)
        
        # Raw sensor string
        self.create_subscription(String, '/sensors/raw', self.update_raw, 10)
        
    def update_ultrasonic(self, position, msg):
        self.ultrasonic_data[position] = msg.range
        self.last_update = datetime.now()
        
    def update_ir_object(self, position, msg):
        self.ir_object_data[position] = msg.data
        self.last_update = datetime.now()
        
    def update_ir_stair(self, position, msg):
        self.ir_stair_data[position] = msg.data
        self.last_update = datetime.now()
        
    def update_encoder(self, side, msg):
        self.encoder_data[side] = msg.data
        self.last_update = datetime.now()
        
    def update_odom(self, msg):
        self.odom_data['x'] = msg.pose.pose.position.x
        self.odom_data['y'] = msg.pose.pose.position.y
        
        # Convert quaternion to yaw
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        self.odom_data['theta'] = math.atan2(2.0 * (qw * qz), 1.0 - 2.0 * (qz * qz))
        
        self.odom_data['linear_vel'] = msg.twist.twist.linear.x
        self.odom_data['angular_vel'] = msg.twist.twist.angular.z
        self.last_update = datetime.now()
        
    def update_lidar(self, msg):
        self.lidar_data['range_count'] = len(msg.ranges)
        valid_ranges = [r for r in msg.ranges if msg.range_min < r < msg.range_max]
        if valid_ranges:
            self.lidar_data['min_range'] = min(valid_ranges)
            self.lidar_data['max_range'] = max(valid_ranges)
        self.lidar_data['scan_time'] = msg.scan_time
        self.last_update = datetime.now()
        
    def update_raw(self, msg):
        self.raw_sensor_data = msg.data
        self.last_update = datetime.now()


def draw_dashboard(stdscr, node):
    """Draw the dashboard using curses"""
    curses.curs_set(0)  # Hide cursor
    stdscr.nodelay(1)   # Non-blocking input
    stdscr.timeout(100) # Refresh every 100ms
    
    # Color pairs
    curses.init_pair(1, curses.COLOR_GREEN, curses.COLOR_BLACK)
    curses.init_pair(2, curses.COLOR_RED, curses.COLOR_BLACK)
    curses.init_pair(3, curses.COLOR_YELLOW, curses.COLOR_BLACK)
    curses.init_pair(4, curses.COLOR_CYAN, curses.COLOR_BLACK)
    curses.init_pair(5, curses.COLOR_MAGENTA, curses.COLOR_BLACK)
    
    while rclpy.ok():
        try:
            stdscr.clear()
            height, width = stdscr.getmaxyx()
            
            # Title
            title = "🤖 FLOOR CLEANING ROBOT - SENSOR DASHBOARD 🤖"
            stdscr.addstr(0, max(0, (width - len(title)) // 2), title, 
                         curses.color_pair(4) | curses.A_BOLD)
            
            timestamp = f"Last Update: {node.last_update.strftime('%H:%M:%S.%f')[:-3]}"
            stdscr.addstr(1, max(0, (width - len(timestamp)) // 2), timestamp)
            stdscr.addstr(2, 0, "=" * min(width - 1, 100))
            
            row = 3
            col_left = 2
            col_right = width // 2 + 2
            
            # Left Column: Ultrasonic Sensors
            stdscr.addstr(row, col_left, "📏 ULTRASONIC SENSORS (Range in cm)", 
                         curses.color_pair(3) | curses.A_BOLD)
            row += 1
            for pos, value in node.ultrasonic_data.items():
                display_val = f"{value * 100:.1f}" if value is not None else "---"
                color = curses.color_pair(1) if value and value > 0.10 else curses.color_pair(2)
                stdscr.addstr(row, col_left + 2, f"{pos:12s}: {display_val:>6s} cm", color)
                row += 1
            
            row += 1
            
            # IR Object Detection
            stdscr.addstr(row, col_left, "🔴 IR OBJECT DETECTION", 
                         curses.color_pair(3) | curses.A_BOLD)
            row += 1
            for pos, value in node.ir_object_data.items():
                status = "DETECTED" if value else "Clear" if value is not None else "---"
                color = curses.color_pair(2) if value else curses.color_pair(1)
                stdscr.addstr(row, col_left + 2, f"{pos:12s}: {status:>10s}", color)
                row += 1
            
            row += 1
            
            # IR Stair Detection
            stdscr.addstr(row, col_left, "⚠️  IR STAIR DETECTION", 
                         curses.color_pair(3) | curses.A_BOLD)
            row += 1
            for pos, value in node.ir_stair_data.items():
                status = "STAIR!" if value else "Safe" if value is not None else "---"
                color = curses.color_pair(2) if value else curses.color_pair(1)
                stdscr.addstr(row, col_left + 2, f"{pos:12s}: {status:>10s}", color)
                row += 1
            
            # Right Column: Encoders
            row = 3
            stdscr.addstr(row, col_right, "⚙️  WHEEL ENCODERS (Ticks)", 
                         curses.color_pair(3) | curses.A_BOLD)
            row += 1
            stdscr.addstr(row, col_right + 2, f"Left:  {node.encoder_data['left']:>8d}", 
                         curses.color_pair(4))
            row += 1
            stdscr.addstr(row, col_right + 2, f"Right: {node.encoder_data['right']:>8d}", 
                         curses.color_pair(4))
            row += 2
            
            # Odometry
            stdscr.addstr(row, col_right, "🧭 ODOMETRY", 
                         curses.color_pair(3) | curses.A_BOLD)
            row += 1
            stdscr.addstr(row, col_right + 2, 
                         f"X:     {node.odom_data['x']:>7.3f} m", curses.color_pair(4))
            row += 1
            stdscr.addstr(row, col_right + 2, 
                         f"Y:     {node.odom_data['y']:>7.3f} m", curses.color_pair(4))
            row += 1
            stdscr.addstr(row, col_right + 2, 
                         f"Theta: {math.degrees(node.odom_data['theta']):>7.1f}°", 
                         curses.color_pair(4))
            row += 1
            stdscr.addstr(row, col_right + 2, 
                         f"Lin V: {node.odom_data['linear_vel']:>7.3f} m/s", 
                         curses.color_pair(4))
            row += 1
            stdscr.addstr(row, col_right + 2, 
                         f"Ang V: {node.odom_data['angular_vel']:>7.3f} rad/s", 
                         curses.color_pair(4))
            row += 2
            
            # LIDAR
            stdscr.addstr(row, col_right, "📡 LIDAR SCAN", 
                         curses.color_pair(3) | curses.A_BOLD)
            row += 1
            stdscr.addstr(row, col_right + 2, 
                         f"Points:    {node.lidar_data['range_count']:>6d}", 
                         curses.color_pair(5))
            row += 1
            stdscr.addstr(row, col_right + 2, 
                         f"Min Range: {node.lidar_data['min_range']:>6.2f} m", 
                         curses.color_pair(5))
            row += 1
            stdscr.addstr(row, col_right + 2, 
                         f"Max Range: {node.lidar_data['max_range']:>6.2f} m", 
                         curses.color_pair(5))
            row += 1
            stdscr.addstr(row, col_right + 2, 
                         f"Scan Time: {node.lidar_data['scan_time']*1000:>6.1f} ms", 
                         curses.color_pair(5))
            
            # Bottom: Raw sensor data
            if height > 25:
                bottom_row = min(height - 3, row + 10)
                stdscr.addstr(bottom_row, 0, "=" * min(width - 1, 100))
                stdscr.addstr(bottom_row + 1, col_left, "📊 RAW SENSOR DATA:", 
                             curses.color_pair(3) | curses.A_BOLD)
                # Truncate if too long
                raw_display = node.raw_sensor_data[:min(width - col_left - 2, 90)]
                stdscr.addstr(bottom_row + 2, col_left, raw_display, curses.color_pair(1))
            
            # Instructions
            stdscr.addstr(height - 1, 0, "Press 'q' to quit | Press 'r' to reset encoders", 
                         curses.A_DIM)
            
            stdscr.refresh()
            
            # Check for input
            key = stdscr.getch()
            if key == ord('q') or key == ord('Q'):
                break
            
            # Spin ROS node
            rclpy.spin_once(node, timeout_sec=0.01)
            
        except KeyboardInterrupt:
            break
        except Exception as e:
            # Handle terminal resize or other errors
            pass


def main(args=None):
    rclpy.init(args=args)
    node = SensorDashboard()
    
    try:
        curses.wrapper(draw_dashboard, node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
