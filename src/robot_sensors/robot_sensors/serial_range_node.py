#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
import serial, time

class SerialUltrasonic(Node):
    def __init__(self):
        super().__init__('serial_ultrasonic')
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('topic', '/ultrasonic/range')
        self.declare_parameter('frame_id', 'ultrasonic_front')
        self.declare_parameter('fov', 0.26)
        self.declare_parameter('min_range', 0.02)
        self.declare_parameter('max_range', 4.00)

        port = self.get_parameter('port').value
        baud = self.get_parameter('baud').value
        topic = self.get_parameter('topic').value
        self.frame = self.get_parameter('frame_id').value
        fov = float(self.get_parameter('fov').value)
        min_r = float(self.get_parameter('min_range').value)
        max_r = float(self.get_parameter('max_range').value)

        self.pub = self.create_publisher(Range, topic, 10)
        self.template = Range()
        self.template.radiation_type = Range.ULTRASOUND
        self.template.field_of_view = fov
        self.template.min_range = min_r
        self.template.max_range = max_r

        self.get_logger().info(f"Opening {port} @ {baud}")
        self.ser = serial.Serial(port, baudrate=baud, timeout=1)
        time.sleep(2.0)  # allow Arduino reset

        self.timer = self.create_timer(0.05, self.read_once)  # 20 Hz

    def read_once(self):
        try:
            line = self.ser.readline().decode('utf-8', errors='ignore').strip()
        except Exception as e:
            self.get_logger().warn(f"Serial read error: {e}")
            return
        if not line:
            return
        try:
            d_cm = float(line)
        except ValueError:
            return

        d_m = max(self.template.min_range,
                  min(d_cm / 100.0, self.template.max_range))

        msg = self.template
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame
        msg.range = float(d_m)
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = SerialUltrasonic()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
