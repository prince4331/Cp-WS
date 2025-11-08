#!/usr/bin/env python3
"""
Multi-Sensor ROS Bridge for Autonomous Floor Cleaning Robot
Reads all sensor data from Arduino Mega and publishes to ROS topics.
Also exposes auxiliary outputs (buzzer, indicator LED).
"""
from math import nan
from typing import Dict, List, Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range, BatteryState, Imu
from std_msgs.msg import Bool, Int32, String
import threading
import serial
import time


def safe_float_list(values: List[str]) -> Optional[List[float]]:
    try:
        return [float(v) for v in values]
    except ValueError:
        return None


class MultiSensorBridge(Node):
    def __init__(self):
        super().__init__('multi_sensor_bridge')

        # Declare parameters
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('battery_voltage_scale', 0.01995)  # volts per ADC count (24V divider 19.1k/6.2k)
        self.declare_parameter('battery_voltage_offset', 0.0)
        self.declare_parameter('battery_current_scale', 0.001)      # amps per ADC count
        self.declare_parameter('battery_current_offset', 0.0)
        self.declare_parameter('battery_temp_scale', 0.1)           # degC per ADC count
        self.declare_parameter('battery_temp_offset', 0.0)
        self.declare_parameter('imu_frame_id', 'imu_link')
        self.declare_parameter('water_clean_frame', 'water_clean_level')
        self.declare_parameter('water_dirty_frame', 'water_dirty_level')

        port = self.get_parameter('port').value
        baud = self.get_parameter('baud').value
        self.batt_voltage_scale = float(self.get_parameter('battery_voltage_scale').value)
        self.batt_voltage_offset = float(self.get_parameter('battery_voltage_offset').value)
        self.batt_current_scale = float(self.get_parameter('battery_current_scale').value)
        self.batt_current_offset = float(self.get_parameter('battery_current_offset').value)
        self.batt_temp_scale = float(self.get_parameter('battery_temp_scale').value)
        self.batt_temp_offset = float(self.get_parameter('battery_temp_offset').value)
        self.imu_frame_id = str(self.get_parameter('imu_frame_id').value)
        self.water_clean_frame = str(self.get_parameter('water_clean_frame').value)
        self.water_dirty_frame = str(self.get_parameter('water_dirty_frame').value)

        # Create publishers for ultrasonic sensors (obstacle array)
        self.pub_us_front = self.create_publisher(Range, '/ultrasonic/front', 10)
        self.pub_us_fright = self.create_publisher(Range, '/ultrasonic/front_right', 10)
        self.pub_us_fleft = self.create_publisher(Range, '/ultrasonic/front_left', 10)
        self.pub_us_right = self.create_publisher(Range, '/ultrasonic/right', 10)
        self.pub_us_left = self.create_publisher(Range, '/ultrasonic/left', 10)
        # Water level ultrasonic sensors
        self.pub_us_clean = self.create_publisher(Range, '/water_level/clean', 10)
        self.pub_us_dirty = self.create_publisher(Range, '/water_level/dirty', 10)

        # IR sensors
        self.pub_ir_fright_obj = self.create_publisher(Bool, '/ir/front_right/object', 10)
        self.pub_ir_fleft_obj = self.create_publisher(Bool, '/ir/front_left/object', 10)
        self.pub_ir_bright_obj = self.create_publisher(Bool, '/ir/back_right/object', 10)
        self.pub_ir_bleft_obj = self.create_publisher(Bool, '/ir/back_left/object', 10)

        self.pub_ir_fright_stair = self.create_publisher(Bool, '/ir/front_right/stair', 10)
        self.pub_ir_fleft_stair = self.create_publisher(Bool, '/ir/front_left/stair', 10)
        self.pub_ir_bright_stair = self.create_publisher(Bool, '/ir/back_right/stair', 10)
        self.pub_ir_bleft_stair = self.create_publisher(Bool, '/ir/back_left/stair', 10)

        # Encoders
        self.pub_enc_left = self.create_publisher(Int32, '/encoder/left', 10)
        self.pub_enc_right = self.create_publisher(Int32, '/encoder/right', 10)

        # Safety IO
        self.pub_estop_main = self.create_publisher(Bool, '/safety/estop_main', 10)
        self.pub_estop_front = self.create_publisher(Bool, '/safety/front_switch', 10)
        self.pub_estop_any = self.create_publisher(Bool, '/safety/estop', 10)

        # Battery & IMU
        self.pub_battery = self.create_publisher(BatteryState, '/battery/state', 10)
        self.pub_imu = self.create_publisher(Imu, '/imu/data', 10)

        # Auxiliary outputs
        self.pub_buzzer_state = self.create_publisher(Bool, '/buzzer/state', 10)
        self.pub_led_state = self.create_publisher(Bool, '/indicator/state', 10)
        self.pub_vacuum_state = self.create_publisher(Bool, '/vacuum/state', 10)
        self.pub_brush_main_state = self.create_publisher(Bool, '/brush/main/state', 10)
        self.pub_brush_left_state = self.create_publisher(Bool, '/brush/left/state', 10)
        self.pub_brush_right_state = self.create_publisher(Bool, '/brush/right/state', 10)

        # Raw data publisher for debugging
        self.pub_raw = self.create_publisher(String, '/sensors/raw', 10)

        # Command subscribers for auxiliary outputs
        self.last_buzzer_cmd: Optional[bool] = None
        self.last_led_cmd: Optional[bool] = None
        self.last_vacuum_cmd: Optional[bool] = None
        self.last_brush_main_cmd: Optional[bool] = None
        self.last_brush_left_cmd: Optional[bool] = None
        self.last_brush_right_cmd: Optional[bool] = None
        self.create_subscription(Bool, '/buzzer/cmd', self.handle_buzzer_cmd, 10)
        self.create_subscription(Bool, '/indicator/cmd', self.handle_led_cmd, 10)
        self.create_subscription(Bool, '/vacuum/cmd', self.handle_vacuum_cmd, 10)
        self.create_subscription(Bool, '/brush/main/cmd', self.handle_brush_main_cmd, 10)
        self.create_subscription(Bool, '/brush/left/cmd', self.handle_brush_left_cmd, 10)
        self.create_subscription(Bool, '/brush/right/cmd', self.handle_brush_right_cmd, 10)
        self.create_subscription(String, '/motor/pwm', self.handle_motor_pwm, 10)  # Changed from manual_pwm

        # Open serial connection
        self.get_logger().info(f"Opening {port} @ {baud}")
        self.ser = serial.Serial(port, baudrate=baud, timeout=1)
        self.serial_write_lock = threading.Lock()
        time.sleep(2.0)  # Allow Arduino reset

        # Create timer for reading sensor data
        self.timer = self.create_timer(0.05, self.read_sensors)  # 20 Hz

        self.get_logger().info("Multi-Sensor Bridge initialized - sensors + aux IO ready")

    def destroy_node(self):
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
        except Exception:
            pass
        super().destroy_node()

    def send_aux_command(self, command: str) -> None:
        """Send a command string to the Arduino (adds newline automatically)."""
        try:
            if self.ser and self.ser.is_open:
                self.ser.write(f"{command}\n".encode('utf-8'))
        except serial.SerialException as exc:
            self.get_logger().warn(f"Failed to send command '{command}': {exc}")

    def handle_buzzer_cmd(self, msg: Bool) -> None:
        if self.last_buzzer_cmd == msg.data:
            return
        self.last_buzzer_cmd = msg.data
        self.send_aux_command(f"BZ:{1 if msg.data else 0}")

    def handle_led_cmd(self, msg: Bool) -> None:
        if self.last_led_cmd == msg.data:
            return
        self.last_led_cmd = msg.data
        self.send_aux_command(f"LED:{1 if msg.data else 0}")

    def handle_motor_pwm(self, msg: String) -> None:
        data = msg.data.strip()
        if not data:
            return

        if data.lower() in {'stop', '0', '0,0'}:
            command = "M:0,0\n"
        else:
            parts = data.split(',')
            if len(parts) != 2:
                self.get_logger().warn(f"Invalid motor PWM payload: {data}")
                return
            try:
                left_pwm = max(-255, min(255, int(parts[0])))
                right_pwm = max(-255, min(255, int(parts[1])))
            except ValueError:
                self.get_logger().warn(f"Invalid motor PWM numeric values: {data}")
                return
            command = f"M:{left_pwm},{right_pwm}\n"

        try:
            with self.serial_write_lock:
                self.ser.write(command.encode('utf-8'))
            self.get_logger().info(f"Sent motor command: {command.strip()}")
        except Exception as exc:
            self.get_logger().warn(f"Failed to send motor command: {exc}")

    def handle_vacuum_cmd(self, msg: Bool) -> None:
        if self.last_vacuum_cmd == msg.data:
            return
        self.last_vacuum_cmd = msg.data
        self.send_aux_command(f"VAC:{1 if msg.data else 0}")

    def handle_brush_main_cmd(self, msg: Bool) -> None:
        if self.last_brush_main_cmd == msg.data:
            return
        self.last_brush_main_cmd = msg.data
        self.send_aux_command(f"BRM:{1 if msg.data else 0}")

    def handle_brush_left_cmd(self, msg: Bool) -> None:
        if self.last_brush_left_cmd == msg.data:
            return
        self.last_brush_left_cmd = msg.data
        self.send_aux_command(f"BRL:{1 if msg.data else 0}")

    def handle_brush_right_cmd(self, msg: Bool) -> None:
        if self.last_brush_right_cmd == msg.data:
            return
        self.last_brush_right_cmd = msg.data
        self.send_aux_command(f"BRR:{1 if msg.data else 0}")

    def read_sensors(self):
        try:
            line = self.ser.readline().decode('utf-8', errors='ignore').strip()
        except Exception as exc:
            self.get_logger().warn(f"Serial read error: {exc}")
            return

        if not line:
            return

        # Publish raw data for logging/diagnostics
        raw_msg = String()
        raw_msg.data = line
        self.pub_raw.publish(raw_msg)

        parts = line.split('|')
        data_map: Dict[str, str] = {}

        for part in parts:
            if ':' not in part:
                continue
            key, value = part.split(':', 1)
            data_map[key] = value

        now = self.get_clock().now().to_msg()

        # Parse ultrasonic obstacle data
        if 'US' in data_map:
            us_values = safe_float_list(data_map['US'].split(','))
            if us_values and len(us_values) >= 5:
                self.publish_ultrasonic(us_values, now)

        # Parse water level ultrasonics
        if 'USW' in data_map:
            usw_values = safe_float_list(data_map['USW'].split(','))
            if usw_values and len(usw_values) >= 2:
                self.publish_water_levels(usw_values, now)

        # Parse IR sensors
        if 'IR_OBJ' in data_map:
            self.publish_ir_object(data_map['IR_OBJ'].split(','))
        if 'IR_STAIR' in data_map:
            self.publish_ir_stair(data_map['IR_STAIR'].split(','))

        # Encoders
        if 'ENC' in data_map:
            self.publish_encoders(data_map['ENC'].split(','))

        # Safety switches
        if 'ESTOP' in data_map:
            self.publish_estop(data_map['ESTOP'].split(','))

        # Battery telemetry
        if 'BAT' in data_map:
            self.publish_battery(data_map['BAT'].split(','), now)

        # IMU data
        if 'IMU' in data_map:
            self.publish_imu(data_map['IMU'].split(','), now)

        # Auxiliary state feedback
        if 'STATE' in data_map:
            self.publish_aux_state(data_map['STATE'].split(','))
        if 'RELAY' in data_map:
            self.publish_relay_state(data_map['RELAY'].split(','))

    def publish_range(self, publisher, frame_id: str, distance_cm: float, timestamp) -> None:
        """Helper to publish a Range message if distance is valid."""
        if distance_cm <= 0:
            return
        msg = Range()
        msg.header.stamp = timestamp
        msg.header.frame_id = frame_id
        msg.radiation_type = Range.ULTRASOUND
        msg.field_of_view = 0.26
        msg.min_range = 0.02
        msg.max_range = 4.0
        msg.range = distance_cm / 100.0  # cm → meters
        publisher.publish(msg)

    def publish_ultrasonic(self, distances: List[float], timestamp) -> None:
        """Publish obstacle ultrasonic array."""
        mapping = [
            (self.pub_us_front, 'ultrasonic_front'),
            (self.pub_us_fright, 'ultrasonic_front_right'),
            (self.pub_us_fleft, 'ultrasonic_front_left'),
            (self.pub_us_right, 'ultrasonic_right'),
            (self.pub_us_left, 'ultrasonic_left'),
        ]
        for idx, (pub, frame) in enumerate(mapping):
            if idx < len(distances):
                self.publish_range(pub, frame, distances[idx], timestamp)

    def publish_water_levels(self, distances: List[float], timestamp) -> None:
        """Publish clean/dirty water level ultrasonic sensors."""
        mapping = [
            (self.pub_us_clean, self.water_clean_frame),
            (self.pub_us_dirty, self.water_dirty_frame),
        ]
        for idx, (pub, frame) in enumerate(mapping):
            if idx < len(distances):
                self.publish_range(pub, frame, distances[idx], timestamp)

    def publish_ir_object(self, data: List[str]) -> None:
        """Publish IR object detection sensor data."""
        if len(data) < 4:
            self.get_logger().warn(f"IR_OBJ payload length {len(data)} < 4: {data}")
            return
        try:
            values = [int(v) for v in data[:4]]
        except ValueError:
            return

        msgs = [
            (self.pub_ir_fright_obj, values[0] == 0),
            (self.pub_ir_fleft_obj, values[1] == 0),
            (self.pub_ir_bright_obj, values[2] == 0),
            (self.pub_ir_bleft_obj, values[3] == 0),
        ]
        for pub, detected in msgs:
            msg = Bool()
            msg.data = detected
            pub.publish(msg)

    def publish_ir_stair(self, data: List[str]) -> None:
        """Publish IR stair detection sensor data."""
        if len(data) < 4:
            self.get_logger().warn(f"IR_STAIR payload length {len(data)} < 4: {data}")
            return
        try:
            values = [int(v) for v in data[:4]]
        except ValueError:
            return

        msgs = [
            (self.pub_ir_fright_stair, values[0] == 0),
            (self.pub_ir_fleft_stair, values[1] == 0),
            (self.pub_ir_bright_stair, values[2] == 0),
            (self.pub_ir_bleft_stair, values[3] == 0),
        ]
        for pub, detected in msgs:
            msg = Bool()
            msg.data = detected
            pub.publish(msg)

    def publish_encoders(self, data: List[str]) -> None:
        """Publish encoder tick counts."""
        if len(data) < 2:
            return
        try:
            left_count = int(data[0])
            right_count = int(data[1])
        except ValueError:
            return

        msg_left = Int32()
        msg_left.data = left_count
        self.pub_enc_left.publish(msg_left)

        msg_right = Int32()
        msg_right.data = right_count
        self.pub_enc_right.publish(msg_right)

    def publish_estop(self, data: List[str]) -> None:
        """Publish emergency stop and front bumper switch states."""
        if len(data) < 2:
            return
        try:
            main_active = int(data[0]) == 1
            front_active = int(data[1]) == 1
        except ValueError:
            return

        msg_main = Bool()
        msg_main.data = main_active
        self.pub_estop_main.publish(msg_main)

        msg_front = Bool()
        msg_front.data = front_active
        self.pub_estop_front.publish(msg_front)

        msg_any = Bool()
        msg_any.data = main_active or front_active
        self.pub_estop_any.publish(msg_any)

    def publish_battery(self, data: List[str], timestamp) -> None:
        """Convert raw ADC readings to BatteryState."""
        if len(data) < 3:
            return
        try:
            raw_voltage = float(data[0])
            raw_current = float(data[1])
            raw_temp = float(data[2])
        except ValueError:
            return

        batt_msg = BatteryState()
        batt_msg.header.stamp = timestamp
        batt_msg.voltage = raw_voltage * self.batt_voltage_scale + self.batt_voltage_offset
        batt_msg.current = raw_current * self.batt_current_scale + self.batt_current_offset
        batt_msg.temperature = raw_temp * self.batt_temp_scale + self.batt_temp_offset
        batt_msg.percentage = nan
        batt_msg.charge = nan
        batt_msg.capacity = nan
        batt_msg.design_capacity = nan
        batt_msg.present = True
        batt_msg.location = 'base_link'
        batt_msg.serial_number = ''
        self.pub_battery.publish(batt_msg)

    def publish_imu(self, data: List[str], timestamp) -> None:
        """Publish BNO055 IMU data."""
        if not data:
            return
        if len(data) == 1 and data[0] == 'NA':
            return
        if len(data) < 10:
            return

        try:
            floats = [float(v) for v in data[:10]]
        except ValueError:
            return

        msg = Imu()
        msg.header.stamp = timestamp
        msg.header.frame_id = self.imu_frame_id
        msg.orientation.w = floats[0]
        msg.orientation.x = floats[1]
        msg.orientation.y = floats[2]
        msg.orientation.z = floats[3]
        msg.orientation_covariance = [0.01, 0.0, 0.0,
                                      0.0, 0.01, 0.0,
                                      0.0, 0.0, 0.01]

        msg.angular_velocity.x = floats[4]
        msg.angular_velocity.y = floats[5]
        msg.angular_velocity.z = floats[6]
        msg.angular_velocity_covariance = [0.02, 0.0, 0.0,
                                           0.0, 0.02, 0.0,
                                           0.0, 0.0, 0.02]

        msg.linear_acceleration.x = floats[7]
        msg.linear_acceleration.y = floats[8]
        msg.linear_acceleration.z = floats[9]
        msg.linear_acceleration_covariance = [0.04, 0.0, 0.0,
                                              0.0, 0.04, 0.0,
                                              0.0, 0.0, 0.04]
        self.pub_imu.publish(msg)

    def publish_aux_state(self, data: List[str]) -> None:
        """Publish buzzer/LED state feedback."""
        if len(data) < 2:
            return
        try:
            buzzer_on = int(data[0]) == 1
            led_on = int(data[1]) == 1
        except ValueError:
            return

        buzzer_msg = Bool()
        buzzer_msg.data = buzzer_on
        self.pub_buzzer_state.publish(buzzer_msg)

        led_msg = Bool()
        led_msg.data = led_on
        self.pub_led_state.publish(led_msg)

    def publish_relay_state(self, data: List[str]) -> None:
        """Publish relay output state feedback."""
        if len(data) < 4:
            return
        try:
            vacuum_on = int(data[0]) == 1
            brush_main_on = int(data[1]) == 1
            brush_left_on = int(data[2]) == 1
            brush_right_on = int(data[3]) == 1
        except ValueError:
            return

        for publisher, state in (
            (self.pub_vacuum_state, vacuum_on),
            (self.pub_brush_main_state, brush_main_on),
            (self.pub_brush_left_state, brush_left_on),
            (self.pub_brush_right_state, brush_right_on),
        ):
            msg = Bool()
            msg.data = state
            publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MultiSensorBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
