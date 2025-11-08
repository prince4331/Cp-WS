#!/usr/bin/env python3
"""
Web dashboard for manual actuator control and raw sensor visibility.
Serves a simple HTML UI and exposes a JSON API backed by ROS 2 topics.
"""

import argparse
import json
import math
import threading
import time
from http import HTTPStatus
from http.server import ThreadingHTTPServer, BaseHTTPRequestHandler
from typing import Any, Dict

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import BatteryState, Imu, Range
from std_msgs.msg import Bool, Int32, String


INDEX_HTML = """<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8"/>
  <meta name="viewport" content="width=device-width, initial-scale=1.0"/>
  <title>Robot Test Dashboard</title>
  <style>
    body { font-family: Arial, sans-serif; margin: 0; padding: 0; background: #f3f4f6; color: #111827; }
    header { background: #1f2937; color: #fff; padding: 1rem 2rem; }
    main { padding: 1rem 2rem 3rem; display: grid; gap: 1.5rem; grid-template-columns: repeat(auto-fit, minmax(320px, 1fr)); }
    section { background: #fff; border-radius: 8px; box-shadow: 0 2px 6px rgba(15, 23, 42, 0.15); padding: 1rem 1.25rem; }
    h1 { margin: 0; font-size: 1.75rem; }
    h2 { font-size: 1.25rem; border-bottom: 1px solid #e5e7eb; padding-bottom: 0.5rem; margin-bottom: 1rem; }
    table { width: 100%; border-collapse: collapse; font-size: 0.95rem; }
    th, td { text-align: left; padding: 0.35rem 0.25rem; border-bottom: 1px solid #f3f4f6; }
    .status-ok { color: #047857; }
    .status-warn { color: #b91c1c; font-weight: bold; }
    .actuator-grid { display: grid; gap: 0.5rem; }
    .actuator-item { display: flex; align-items: center; justify-content: space-between; background: #f9fafb; border: 1px solid #e5e7eb; border-radius: 6px; padding: 0.5rem 0.75rem; }
    .switch { position: relative; display: inline-block; width: 42px; height: 22px; }
    .switch input { opacity: 0; width: 0; height: 0; }
    .slider { position: absolute; cursor: pointer; top: 0; left: 0; right: 0; bottom: 0; background-color: #d1d5db; transition: .2s; border-radius: 999px;}
    .slider:before { position: absolute; content: ""; height: 18px; width: 18px; left: 2px; bottom: 2px; background-color: white; transition: .2s; border-radius: 50%; }
    input:checked + .slider { background-color: #2563eb; }
    input:checked + .slider:before { transform: translateX(20px); }
    .cmd-form { display: grid; gap: 0.5rem; }
    .cmd-row { display: flex; gap: 0.5rem; align-items: center; }
    .cmd-row label { min-width: 120px; }
    .cmd-row input { flex: 1; padding: 0.35rem; border-radius: 4px; border: 1px solid #d1d5db; }
    button { background: #2563eb; border: none; color: #fff; padding: 0.5rem 0.75rem; border-radius: 4px; cursor: pointer; }
    button:hover { background: #1d4ed8; }
    .motor-buttons { display: grid; grid-template-columns: repeat(2, minmax(0, 1fr)); gap: 0.5rem; margin-top: 0.5rem; }
    .motor-buttons button { width: 100%; padding: 0.45rem 0.6rem; }
    pre { background: #1f2937; color: #f9fafb; padding: 0.75rem; border-radius: 6px; overflow-x: auto; font-size: 0.85rem; }
    .badge { display: inline-block; padding: 0.2rem 0.5rem; border-radius: 999px; font-size: 0.75rem; }
    .badge.ok { background: #d1fae5; color: #065f46; }
    .badge.warn { background: #fee2e2; color: #991b1b; }
  </style>
</head>
<body>
  <header>
    <h1>Autonomous Floor Cleaning Robot — Test Dashboard</h1>
    <p id="connection-status">Loading...</p>
  </header>
  <main>
    <section>
      <h2>Safety &amp; Status</h2>
      <table>
        <tbody>
          <tr><th>Main E-Stop</th><td id="estop-main"></td></tr>
          <tr><th>Front Bumper</th><td id="front-switch"></td></tr>
          <tr><th>Aggregated Safety</th><td id="estop-any"></td></tr>
          <tr><th>Battery Voltage</th><td id="battery-voltage"></td></tr>
          <tr><th>Battery Current</th><td id="battery-current"></td></tr>
          <tr><th>Battery Temp</th><td id="battery-temp"></td></tr>
        </tbody>
      </table>
    </section>

    <section>
      <h2>Ultrasonic &amp; Tanks</h2>
      <table id="ultrasonic-table"></table>
    </section>

    <section>
      <h2>IR Sensors</h2>
      <table id="ir-table"></table>
    </section>

    <section>
      <h2>Encoders &amp; Odometry</h2>
      <table>
        <tbody>
          <tr><th>Encoder Left</th><td id="enc-left"></td></tr>
          <tr><th>Encoder Right</th><td id="enc-right"></td></tr>
          <tr><th>Position (x, y)</th><td id="odom-pos"></td></tr>
          <tr><th>Heading (yaw)</th><td id="odom-yaw"></td></tr>
          <tr><th>Velocity (lin, ang)</th><td id="odom-vel"></td></tr>
        </tbody>
      </table>
    </section>

    <section>
      <h2>Actuator Controls</h2>
      <div class="actuator-grid">
        <div class="actuator-item">
          <span>Vacuum</span>
          <label class="switch"><input type="checkbox" data-actuator="vacuum"><span class="slider"></span></label>
        </div>
        <div class="actuator-item">
          <span>Main Brush</span>
          <label class="switch"><input type="checkbox" data-actuator="brush_main"><span class="slider"></span></label>
        </div>
        <div class="actuator-item">
          <span>Left Side Brush</span>
          <label class="switch"><input type="checkbox" data-actuator="brush_left"><span class="slider"></span></label>
        </div>
        <div class="actuator-item">
          <span>Right Side Brush</span>
          <label class="switch"><input type="checkbox" data-actuator="brush_right"><span class="slider"></span></label>
        </div>
        <div class="actuator-item">
          <span>Buzzer</span>
          <label class="switch"><input type="checkbox" data-actuator="buzzer"><span class="slider"></span></label>
        </div>
        <div class="actuator-item">
          <span>Status LED</span>
          <label class="switch"><input type="checkbox" data-actuator="indicator"><span class="slider"></span></label>
        </div>
      </div>
      <hr style="margin:1rem 0;">
      <div class="cmd-form">
        <h3>Manual Motion Command</h3>
        <div class="cmd-row">
          <label for="linear-input">Linear X (m/s)</label>
          <input id="linear-input" type="number" value="0.0" step="0.05"/>
        </div>
        <div class="cmd-row">
          <label for="angular-input">Angular Z (rad/s)</label>
          <input id="angular-input" type="number" value="0.0" step="0.05"/>
        </div>
        <div class="cmd-row">
          <button id="send-twist">Send cmd_vel</button>
          <button id="stop-twist" type="button">Stop</button>
        </div>
      </div>
      <hr style="margin:1rem 0;">
      <div class="cmd-form">
        <h3>Manual Motor Test</h3>
        <p style="font-size:0.8rem;color:#6b7280;">Full-speed checks for each wheel. Stop resets both sides.</p>
        <div class="motor-buttons">
          <button type="button" data-motor-btn="left_fwd">Left FWD</button>
          <button type="button" data-motor-btn="right_fwd">Right FWD</button>
          <button type="button" data-motor-btn="left_rev">Left REV</button>
          <button type="button" data-motor-btn="right_rev">Right REV</button>
        </div>
        <div class="cmd-row" style="margin-top:0.5rem;">
          <button id="stop-motor" type="button">Stop Motors</button>
        </div>
      </div>
    </section>

    <section>
      <h2>IMU Orientation</h2>
      <table>
        <tbody>
          <tr><th>Quaternion</th><td id="imu-orientation"></td></tr>
          <tr><th>Gyro (rad/s)</th><td id="imu-gyro"></td></tr>
          <tr><th>Linear Accel (m/s²)</th><td id="imu-accel"></td></tr>
        </tbody>
      </table>
    </section>

    <section style="grid-column: 1 / -1;">
      <h2>Raw Sensor Frame</h2>
      <pre id="raw-data"></pre>
    </section>
  </main>
  <script>
    const ACTUATOR_ENDPOINT = '/api/actuators';
    const STATE_ENDPOINT = '/api/state';
    const statusEl = document.getElementById('connection-status');
    const ultrasonicTable = document.getElementById('ultrasonic-table');
    const irTable = document.getElementById('ir-table');

    async function fetchState() {
      try {
        const res = await fetch(STATE_ENDPOINT);
        if (!res.ok) throw new Error('HTTP ' + res.status);
        const data = await res.json();
        updateUI(data);
        statusEl.textContent = 'Connected — updated ' + new Date(data.timestamp * 1000).toLocaleTimeString();
      } catch (err) {
        statusEl.textContent = 'Connection error: ' + err;
      }
    }

    function badge(text, ok) {
      return `<span class="badge ${ok ? 'ok' : 'warn'}">${text}</span>`;
    }

    function metersToCm(m) {
      if (m === null || m === undefined) return '—';
      return (m * 100).toFixed(1) + ' cm';
    }

    function updateTable(table, entries, formatter) {
      const rows = Object.entries(entries || {}).map(([key, value]) => {
        return `<tr><th>${key}</th><td>${formatter(value)}</td></tr>`;
      }).join('');
      table.innerHTML = rows || '<tr><td colspan="2">No data yet</td></tr>';
    }

    function updateUI(data) {
      document.getElementById('estop-main').innerHTML = badge(data.safety.estop_main ? 'TRIPPED' : 'OK', !data.safety.estop_main);
      document.getElementById('front-switch').innerHTML = badge(data.safety.front_switch ? 'TRIPPED' : 'OK', !data.safety.front_switch);
      document.getElementById('estop-any').innerHTML = badge(data.safety.estop ? 'STOPPED' : 'RUNNING', !data.safety.estop);

      document.getElementById('battery-voltage').textContent = data.battery.voltage ? data.battery.voltage.toFixed(2) + ' V' : '—';
      document.getElementById('battery-current').textContent = data.battery.current ? data.battery.current.toFixed(2) + ' A' : '—';
      document.getElementById('battery-temp').textContent = data.battery.temperature ? data.battery.temperature.toFixed(1) + ' °C' : '—';

      updateTable(ultrasonicTable, data.ultrasonic, metersToCm);
      updateTable(irTable, data.ir, value => value ? badge('ACTIVE', false) : badge('CLEAR', true));

      document.getElementById('enc-left').textContent = data.encoders.left ?? '—';
      document.getElementById('enc-right').textContent = data.encoders.right ?? '—';
      document.getElementById('odom-pos').textContent = data.odom.position ? `${data.odom.position.x.toFixed(3)}, ${data.odom.position.y.toFixed(3)}` : '—';
      document.getElementById('odom-yaw').textContent = data.odom.yaw !== null ? data.odom.yaw.toFixed(2) + ' rad' : '—';
      document.getElementById('odom-vel').textContent = data.odom.velocity ? `${data.odom.velocity.linear.toFixed(3)}, ${data.odom.velocity.angular.toFixed(3)}` : '—';

      document.getElementById('imu-orientation').textContent = data.imu.orientation ? JSON.stringify(data.imu.orientation) : '—';
      document.getElementById('imu-gyro').textContent = data.imu.gyro ? JSON.stringify(data.imu.gyro) : '—';
      document.getElementById('imu-accel').textContent = data.imu.linear_accel ? JSON.stringify(data.imu.linear_accel) : '—';

      document.getElementById('raw-data').textContent = data.raw || '';

      document.querySelectorAll('[data-actuator]').forEach(el => {
        const key = el.dataset.actuator;
        if (key in data.actuators) {
          el.checked = Boolean(data.actuators[key]);
        }
      });
    }

    async function sendActuatorCommand(payload) {
      await fetch(ACTUATOR_ENDPOINT, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(payload)
      });
    }

    document.querySelectorAll('[data-actuator]').forEach(el => {
      el.addEventListener('change', () => {
        const key = el.dataset.actuator;
        sendActuatorCommand({ [key]: el.checked });
      });
    });

    document.getElementById('send-twist').addEventListener('click', async () => {
      const lin = parseFloat(document.getElementById('linear-input').value) || 0.0;
      const ang = parseFloat(document.getElementById('angular-input').value) || 0.0;
      await sendActuatorCommand({ cmd_vel: { linear_x: lin, angular_z: ang } });
    });
    document.getElementById('stop-twist').addEventListener('click', async () => {
      await sendActuatorCommand({ cmd_vel: { linear_x: 0.0, angular_z: 0.0 } });
      document.getElementById('linear-input').value = '0.0';
      document.getElementById('angular-input').value = '0.0';
    });
    document.getElementById('stop-motor').addEventListener('click', async () => {
      await sendActuatorCommand({ motor_pwm: { left: 0, right: 0 } });
    });
    document.querySelectorAll('[data-motor-btn]').forEach(btn => {
      btn.addEventListener('click', async () => {
        const dir = btn.getAttribute('data-motor-btn');
        let left = 0;
        let right = 0;
        switch (dir) {
          case 'left_fwd':
            left = 255;
            break;
          case 'left_rev':
            left = -255;
            break;
          case 'right_fwd':
            right = 255;
            break;
          case 'right_rev':
            right = -255;
            break;
          default:
            break;
        }
        document.getElementById('left-pwm-input').value = String(left);
        document.getElementById('right-pwm-input').value = String(right);
        await sendActuatorCommand({ motor_pwm: { left, right } });
      });
    });

    fetchState();
    setInterval(fetchState, 1000);
  </script>
</body>
</html>
"""


def _clean_float(value: float) -> float | None:
    if value is None:
        return None
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return float(value)


class DashboardNode(Node):
    """ROS node that tracks the latest sensor/actuator state."""

    def __init__(self):
        super().__init__('web_dashboard_node')
        self._lock = threading.Lock()
        self.state: Dict[str, Any] = {
            'timestamp': time.time(),
            'ultrasonic': {},
            'ir': {},
            'encoders': {'left': None, 'right': None},
            'safety': {'estop_main': False, 'front_switch': False, 'estop': False},
            'battery': {'voltage': None, 'current': None, 'temperature': None},
            'imu': {'orientation': None, 'gyro': None, 'linear_accel': None},
            'odom': {'position': None, 'yaw': None, 'velocity': None},
            'actuators': {
                'vacuum': False,
                'brush_main': False,
                'brush_left': False,
                'brush_right': False,
                'buzzer': False,
                'indicator': False,
            },
            'raw': '',
        }

        # Sensor subscriptions
        range_topics = {
            '/ultrasonic/front': ('ultrasonic', 'front'),
            '/ultrasonic/front_left': ('ultrasonic', 'front_left'),
            '/ultrasonic/front_right': ('ultrasonic', 'front_right'),
            '/ultrasonic/left': ('ultrasonic', 'left'),
            '/ultrasonic/right': ('ultrasonic', 'right'),
            '/water_level/clean': ('ultrasonic', 'tank_clean'),
            '/water_level/dirty': ('ultrasonic', 'tank_dirty'),
        }
        for topic, (group, name) in range_topics.items():
            self.create_subscription(
                Range, topic,
                lambda msg, g=group, n=name: self._handle_range(g, n, msg), 10)

        bool_topics = {
            '/ir/front_right/object': ('ir', 'front_right_object'),
            '/ir/front_left/object': ('ir', 'front_left_object'),
            '/ir/back_right/object': ('ir', 'back_right_object'),
            '/ir/back_left/object': ('ir', 'back_left_object'),
            '/ir/front_right/stair': ('ir', 'front_right_stair'),
            '/ir/front_left/stair': ('ir', 'front_left_stair'),
            '/ir/back_right/stair': ('ir', 'back_right_stair'),
            '/ir/back_left/stair': ('ir', 'back_left_stair'),
            '/safety/estop_main': ('safety', 'estop_main'),
            '/safety/front_switch': ('safety', 'front_switch'),
            '/safety/estop': ('safety', 'estop'),
            '/vacuum/state': ('actuators', 'vacuum'),
            '/brush/main/state': ('actuators', 'brush_main'),
            '/brush/left/state': ('actuators', 'brush_left'),
            '/brush/right/state': ('actuators', 'brush_right'),
            '/buzzer/state': ('actuators', 'buzzer'),
            '/indicator/state': ('actuators', 'indicator'),
        }
        for topic, (group, name) in bool_topics.items():
            self.create_subscription(
                Bool, topic,
                lambda msg, g=group, n=name: self._handle_bool(g, n, msg), 10)

        self.create_subscription(
            Int32, '/encoder/left',
            lambda msg: self._handle_encoder('left', msg), 10)
        self.create_subscription(
            Int32, '/encoder/right',
            lambda msg: self._handle_encoder('right', msg), 10)

        self.create_subscription(
            BatteryState, '/battery/state', self._handle_battery, 10)
        self.create_subscription(
            Imu, '/imu/data', self._handle_imu, 10)
        self.create_subscription(
            Odometry, '/odom', self._handle_odom, 10)
        self.create_subscription(
            String, '/sensors/raw', self._handle_raw, 10)

        # Actuator publishers
        self._actuator_publishers = {
            'vacuum': self.create_publisher(Bool, '/vacuum/cmd', 10),
            'brush_main': self.create_publisher(Bool, '/brush/main/cmd', 10),
            'brush_left': self.create_publisher(Bool, '/brush/left/cmd', 10),
            'brush_right': self.create_publisher(Bool, '/brush/right/cmd', 10),
            'buzzer': self.create_publisher(Bool, '/buzzer/cmd', 10),
            'indicator': self.create_publisher(Bool, '/indicator/cmd', 10),
        }
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.motor_pwm_publisher = self.create_publisher(String, '/motor/manual_pwm', 10)

    def _update_state(self, group: str, key: str, value: Any) -> None:
        with self._lock:
            if group in self.state and isinstance(self.state[group], dict):
                self.state[group][key] = value
            else:
                self.state[group] = {key: value}
            self.state['timestamp'] = time.time()

    def _handle_range(self, group: str, name: str, msg: Range) -> None:
        value = _clean_float(msg.range)
        self._update_state(group, name, value)

    def _handle_bool(self, group: str, name: str, msg: Bool) -> None:
        self._update_state(group, name, bool(msg.data))

    def _handle_encoder(self, side: str, msg: Int32) -> None:
        self._update_state('encoders', side, int(msg.data))

    def _handle_battery(self, msg: BatteryState) -> None:
        data = {
            'voltage': _clean_float(msg.voltage),
            'current': _clean_float(msg.current),
            'temperature': _clean_float(msg.temperature),
        }
        self._update_state('battery', 'voltage', data['voltage'])
        self._update_state('battery', 'current', data['current'])
        self._update_state('battery', 'temperature', data['temperature'])

    def _handle_imu(self, msg: Imu) -> None:
        imu_state = {
            'orientation': {
                'x': _clean_float(msg.orientation.x),
                'y': _clean_float(msg.orientation.y),
                'z': _clean_float(msg.orientation.z),
                'w': _clean_float(msg.orientation.w),
            },
            'gyro': {
                'x': _clean_float(msg.angular_velocity.x),
                'y': _clean_float(msg.angular_velocity.y),
                'z': _clean_float(msg.angular_velocity.z),
            },
            'linear_accel': {
                'x': _clean_float(msg.linear_acceleration.x),
                'y': _clean_float(msg.linear_acceleration.y),
                'z': _clean_float(msg.linear_acceleration.z),
            },
        }
        with self._lock:
            self.state['imu'] = imu_state
            self.state['timestamp'] = time.time()

    def _handle_odom(self, msg: Odometry) -> None:
        pose = msg.pose.pose
        twist = msg.twist.twist
        qw = pose.orientation.w
        qz = pose.orientation.z
        qx = pose.orientation.x
        qy = pose.orientation.y
        # Convert quaternion to yaw
        yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
        odom_state = {
            'position': {
                'x': _clean_float(pose.position.x),
                'y': _clean_float(pose.position.y),
            },
            'yaw': _clean_float(yaw),
            'velocity': {
                'linear': _clean_float(twist.linear.x),
                'angular': _clean_float(twist.angular.z),
            },
        }
        with self._lock:
            self.state['odom'] = odom_state
            self.state['timestamp'] = time.time()

    def _handle_raw(self, msg: String) -> None:
        with self._lock:
            self.state['raw'] = msg.data
            self.state['timestamp'] = time.time()

    def get_state_snapshot(self) -> Dict[str, Any]:
        with self._lock:
            # Deep-copy primitives
            snapshot = json.loads(json.dumps(self.state))
        # Ensure all expected groups exist
        snapshot.setdefault('ultrasonic', {})
        snapshot.setdefault('ir', {})
        snapshot.setdefault('encoders', {'left': None, 'right': None})
        snapshot.setdefault('safety', {'estop_main': False, 'front_switch': False, 'estop': False})
        snapshot.setdefault('battery', {'voltage': None, 'current': None, 'temperature': None})
        snapshot.setdefault('imu', {'orientation': None, 'gyro': None, 'linear_accel': None})
        snapshot.setdefault('odom', {'position': None, 'yaw': None, 'velocity': None})
        snapshot.setdefault('actuators', {})
        snapshot.setdefault('raw', '')
        return snapshot

    def handle_actuator_payload(self, payload: Dict[str, Any]) -> None:
        for key, publisher in self._actuator_publishers.items():
            if key in payload:
                msg = Bool()
                msg.data = bool(payload[key])
                publisher.publish(msg)
                self.get_logger().info(f'Set {key} -> {msg.data}')

        if 'cmd_vel' in payload:
            cmd = payload['cmd_vel'] or {}
            twist = Twist()
            twist.linear.x = float(cmd.get('linear_x', 0.0) or 0.0)
            twist.angular.z = float(cmd.get('angular_z', 0.0) or 0.0)
            self.cmd_vel_publisher.publish(twist)
            self.get_logger().info(
                f'Sent cmd_vel linear_x={twist.linear.x:.3f}, angular_z={twist.angular.z:.3f}')
        if 'motor_pwm' in payload:
            cmd = payload['motor_pwm'] or {}
            left_pwm = int(cmd.get('left', 0) or 0)
            right_pwm = int(cmd.get('right', 0) or 0)
            msg = String()
            msg.data = f"{left_pwm},{right_pwm}"
            self.motor_pwm_publisher.publish(msg)
            self.get_logger().info(f'Sent manual motor PWM L={left_pwm} R={right_pwm}')


dashboard_node: DashboardNode | None = None


class DashboardRequestHandler(BaseHTTPRequestHandler):
    """HTTP handler that serves static UI and JSON API endpoints."""

    def _write_json(self, data: Dict[str, Any], status: HTTPStatus = HTTPStatus.OK) -> None:
        payload = json.dumps(data).encode('utf-8')
        self.send_response(status)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(payload)))
        self.end_headers()
        self.wfile.write(payload)

    def do_GET(self) -> None:  # noqa: N802 (BaseHTTPRequestHandler requirement)
        global dashboard_node
        if self.path == '/' or self.path == '/index.html':
            content = INDEX_HTML.encode('utf-8')
            self.send_response(HTTPStatus.OK)
            self.send_header('Content-Type', 'text/html; charset=utf-8')
            self.send_header('Content-Length', str(len(content)))
            self.end_headers()
            self.wfile.write(content)
        elif self.path == '/api/state':
            if not dashboard_node:
                self._write_json({'error': 'ros node not ready'}, HTTPStatus.SERVICE_UNAVAILABLE)
                return
            snapshot = dashboard_node.get_state_snapshot()
            self._write_json(snapshot)
        else:
            self._write_json({'error': 'not found'}, HTTPStatus.NOT_FOUND)

    def do_POST(self) -> None:  # noqa: N802
        global dashboard_node
        length = int(self.headers.get('Content-Length', '0') or '0')
        body = self.rfile.read(length) if length > 0 else b'{}'
        if not dashboard_node:
            self._write_json({'error': 'ros node not ready'}, HTTPStatus.SERVICE_UNAVAILABLE)
            return
        try:
            payload = json.loads(body.decode('utf-8') or '{}')
        except json.JSONDecodeError:
            self._write_json({'error': 'invalid json'}, HTTPStatus.BAD_REQUEST)
            return

        if self.path == '/api/actuators':
            dashboard_node.handle_actuator_payload(payload)
            self._write_json({'status': 'ok'})
        else:
            self._write_json({'error': 'not found'}, HTTPStatus.NOT_FOUND)

    def log_message(self, format: str, *args: Any) -> None:
        # Silence default logging to keep CLI clean.
        return


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(description='Web dashboard for robot sensors and actuators.')
    parser.add_argument('--host', default='0.0.0.0', help='Interface to bind the HTTP server to.')
    parser.add_argument('--port', type=int, default=8080, help='Port for the HTTP server.')
    args = parser.parse_args(argv)

    rclpy.init(args=None)
    node = DashboardNode()
    global dashboard_node
    dashboard_node = node

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    ros_thread = threading.Thread(target=executor.spin, daemon=True)
    ros_thread.start()

    server = ThreadingHTTPServer((args.host, args.port), DashboardRequestHandler)
    try:
        node.get_logger().info(f'Web dashboard available at http://{args.host}:{args.port}/')
        server.serve_forever()
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down web dashboard...')
    finally:
        server.shutdown()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
