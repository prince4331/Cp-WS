#!/usr/bin/env python3
"""
Comprehensive Robot Dashboard
Monitors all sensors and controls all actuators for the autonomous                elif part.startswith('RELAY:'):
                    values = part[6:].split(',')
                    if len(values) >= 4:
                        self.sensor_data['relays'] = {
                            'vacuum': bool(int(values[0])),
                            'brush_main': bool(int(values[1])),
                            'brush_left': bool(int(values[2])),
                            'brush_right': bool(int(values[3]))
                        }
                elif part.startswith('STATE:'):
                    values = part[6:].split(',')
                    if len(values) >= 2:
                        self.sensor_data['state'] = {
                            'buzzer': bool(int(values[0])),
                            'led': bool(int(values[1]))
                        }
            self.last_update = time.time()
        except Exception as e:
            self.get_logger().error(f'Error parsing sensor data: {e}')bot
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool
from sensor_msgs.msg import LaserScan, Imu
from flask import Flask, render_template_string, jsonify, request
import threading
import time
import json

class RobotDashboard(Node):
    def __init__(self):
        super().__init__('robot_dashboard')
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        # Individual publishers for actuators/indicators
        self.buzzer_pub = self.create_publisher(Bool, '/buzzer/cmd', 10)
        self.led_pub = self.create_publisher(Bool, '/indicator/cmd', 10)
        self.vacuum_pub = self.create_publisher(Bool, '/vacuum/cmd', 10)
        self.brush_main_pub = self.create_publisher(Bool, '/brush/main/cmd', 10)
        self.brush_left_pub = self.create_publisher(Bool, '/brush/left/cmd', 10)
        self.brush_right_pub = self.create_publisher(Bool, '/brush/right/cmd', 10)
        
        # Subscribers
        self.create_subscription(String, '/sensors/raw', self.sensor_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        
        # Data storage
        self.sensor_data = {
            'ultrasonic': {'front': 0, 'front_right': 0, 'front_left': 0, 'right': 0, 'left': 0},
            'water_tanks': {'clean': 0, 'dirty': 0},
            'ir_object': {'front_right': 0, 'front_left': 0, 'back_right': 0, 'back_left': 0},
            'ir_stair': {'front_right': 0, 'front_left': 0, 'back_right': 0, 'back_left': 0},
            'encoders': {'left': 0, 'right': 0},
            'safety': {'pin22': 'N/A', 'pin24': 'N/A', 'safety_active': 0, 'status': 'SAFE'},
            'battery': {'voltage': 0, 'current': 0, 'temperature': 0},
            'imu': {'qw': 0, 'qx': 0, 'qy': 0, 'qz': 0, 'gx': 0, 'gy': 0, 'gz': 0, 'mx': 0, 'my': 0, 'mz': 0},
            'relays': {'vacuum': False, 'brush_main': False, 'brush_left': False, 'brush_right': False},
            'state': {'buzzer': False, 'led': False},
            'lidar': {'ranges': [], 'min_range': 0, 'max_range': 0}
        }
        self.last_update = time.time()
        
    def sensor_callback(self, msg):
        """Parse sensor data from Arduino"""
        try:
            parts = msg.data.split('|')
            for part in parts:
                if part.startswith('US:'):
                    values = part[3:].split(',')
                    if len(values) >= 5:
                        self.sensor_data['ultrasonic'] = {
                            'front': float(values[0]),
                            'front_right': float(values[1]),
                            'front_left': float(values[2]),
                            'right': float(values[3]),
                            'left': float(values[4])
                        }
                elif part.startswith('USW:'):
                    values = part[4:].split(',')
                    if len(values) >= 2:
                        self.sensor_data['water_tanks'] = {
                            'clean': float(values[0]),
                            'dirty': float(values[1])
                        }
                elif part.startswith('IR_OBJ:'):
                    values = part[7:].split(',')
                    if len(values) >= 4:
                        self.sensor_data['ir_object'] = {
                            'front_right': int(values[0]),
                            'front_left': int(values[1]),
                            'back_right': int(values[2]),
                            'back_left': int(values[3])
                        }
                elif part.startswith('IR_STAIR:'):
                    values = part[9:].split(',')
                    if len(values) >= 4:
                        self.sensor_data['ir_stair'] = {
                            'front_right': int(values[0]),
                            'front_left': int(values[1]),
                            'back_right': int(values[2]),
                            'back_left': int(values[3])
                        }
                elif part.startswith('ENC:'):
                    values = part[4:].split(',')
                    if len(values) >= 2:
                        self.sensor_data['encoders'] = {
                            'left': int(values[0]),
                            'right': int(values[1])
                        }
                elif part.startswith('ESTOP:'):
                    values = part[6:].split(',')
                    if len(values) >= 3:
                        pin22 = int(values[0])
                        pin24 = int(values[1])
                        safety_active = int(values[2])
                        self.sensor_data['safety'] = {
                            'pin22': pin22,
                            'pin24': pin24,
                            'safety_active': safety_active,
                            'status': 'EMERGENCY STOP' if safety_active else 'SAFE'
                        }
                elif part.startswith('BAT:'):
                    values = part[4:].split(',')
                    if len(values) >= 3:
                        self.sensor_data['battery'] = {
                            'voltage': int(values[0]),
                            'current': int(values[1]),
                            'temperature': int(values[2])
                        }
                elif part.startswith('IMU:'):
                    values = part[4:].split(',')
                    if len(values) >= 10:  # Now we parse all 10 values
                        self.sensor_data['imu'] = {
                            'qw': float(values[0]),
                            'qx': float(values[1]),
                            'qy': float(values[2]),
                            'qz': float(values[3]),
                            'gx': float(values[4]),
                            'gy': float(values[5]),
                            'gz': float(values[6]),
                            'mx': float(values[7]),  # Magnetometer X
                            'my': float(values[8]),  # Magnetometer Y
                            'mz': float(values[9])   # Magnetometer Z
                        }
                    elif len(values) >= 7:  # Fallback if mag data missing
                        self.sensor_data['imu'] = {
                            'qw': float(values[0]),
                            'qx': float(values[1]),
                            'qy': float(values[2]),
                            'qz': float(values[3]),
                            'gx': float(values[4]),
                            'gy': float(values[5]),
                            'gz': float(values[6]),
                            'mx': 0.0,
                            'my': 0.0,
                            'mz': 0.0
                        }
                elif part.startswith('RELAY:'):
                    values = part[6:].split(',')
                    if len(values) >= 4:
                        self.sensor_data['relays'] = {
                            'vacuum': bool(int(values[0])),
                            'brush_main': bool(int(values[1])),
                            'brush_left': bool(int(values[2])),
                            'brush_right': bool(int(values[3]))
                        }
            self.last_update = time.time()
        except Exception as e:
            self.get_logger().error(f'Error parsing sensor data: {e}')
    
    def lidar_callback(self, msg):
        """Update LiDAR data"""
        self.sensor_data['lidar'] = {
            'ranges': msg.ranges[::10],  # Sample every 10th point to reduce data
            'min_range': min(msg.ranges) if msg.ranges else 0,
            'max_range': max(msg.ranges) if msg.ranges else 0
        }
    
    def send_velocity(self, linear_x, angular_z):
        """Send velocity command"""
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self.cmd_vel_pub.publish(msg)
        
    def send_relay_command(self, relay_name, state):
        """Send relay control command"""
        msg = Bool()
        msg.data = bool(state)
        
        if relay_name == 'vacuum':
            self.vacuum_pub.publish(msg)
        elif relay_name == 'brush_main':
            self.brush_main_pub.publish(msg)
        elif relay_name == 'brush_left':
            self.brush_left_pub.publish(msg)
        elif relay_name == 'brush_right':
            self.brush_right_pub.publish(msg)
    
    def send_aux_command(self, device, state):
        """Send buzzer/LED command"""
        msg = Bool()
        msg.data = bool(state)
        
        if device == 'BZ':
            self.buzzer_pub.publish(msg)
        elif device == 'LED':
            self.led_pub.publish(msg)

# Flask web app
app = Flask(__name__)
dashboard_node = None

HTML_TEMPLATE = """
<!DOCTYPE html>
<html>
<head>
    <title>Robot Control Dashboard</title>
    <meta charset="utf-8">
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        * { margin: 0; padding: 0; box-sizing: border-box; }
        body {
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            color: #333;
            padding: 20px;
        }
        .container { max-width: 1400px; margin: 0 auto; }
        h1 {
            text-align: center;
            color: white;
            margin-bottom: 30px;
            font-size: 2.5em;
            text-shadow: 2px 2px 4px rgba(0,0,0,0.3);
        }
        .grid {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(350px, 1fr));
            gap: 20px;
            margin-bottom: 20px;
        }
        .card {
            background: white;
            border-radius: 15px;
            padding: 20px;
            box-shadow: 0 10px 30px rgba(0,0,0,0.2);
        }
        .card h2 {
            color: #667eea;
            margin-bottom: 15px;
            border-bottom: 2px solid #667eea;
            padding-bottom: 10px;
        }
        .sensor-value {
            display: flex;
            justify-content: space-between;
            padding: 8px;
            margin: 5px 0;
            background: #f7fafc;
            border-radius: 5px;
        }
        .sensor-label { font-weight: 600; color: #4a5568; }
        .sensor-data { color: #2d3748; font-family: monospace; }
        .status-safe { color: #48bb78; font-weight: bold; }
        .status-emergency { color: #f56565; font-weight: bold; animation: blink 1s infinite; }
        @keyframes blink { 0%, 50% { opacity: 1; } 51%, 100% { opacity: 0.3; } }
        
        .control-section {
            background: white;
            border-radius: 15px;
            padding: 20px;
            margin-bottom: 20px;
            box-shadow: 0 10px 30px rgba(0,0,0,0.2);
        }
        .button-grid {
            display: grid;
            grid-template-columns: repeat(3, 1fr);
            gap: 10px;
            margin-top: 15px;
        }
        button {
            padding: 15px;
            font-size: 1em;
            border: none;
            border-radius: 10px;
            cursor: pointer;
            font-weight: 600;
            transition: all 0.3s;
            color: white;
        }
        button:hover { transform: scale(1.05); box-shadow: 0 5px 15px rgba(0,0,0,0.3); }
        button:active { transform: scale(0.95); }
        .btn-forward { background: #48bb78; }
        .btn-backward { background: #ed8936; }
        .btn-left { background: #4299e1; }
        .btn-right { background: #9f7aea; }
        .btn-stop { background: #f56565; grid-column: 1 / -1; font-size: 1.2em; }
        .btn-relay { background: #667eea; padding: 10px; margin: 5px 0; }
        .btn-relay.active { background: #48bb78; }
        
        .update-time {
            text-align: center;
            color: white;
            margin-top: 20px;
            font-size: 0.9em;
        }
        
        .ir-indicator {
            display: inline-block;
            width: 12px;
            height: 12px;
            border-radius: 50%;
            margin-left: 10px;
        }
        .ir-clear { background: #48bb78; }
        .ir-blocked { background: #f56565; }
    </style>
</head>
<body>
    <div class="container">
        <h1>🤖 Autonomous Cleaning Robot Dashboard</h1>
        
        <!-- Motor Control -->
        <div class="control-section">
            <h2>🎮 Motor Control</h2>
            <div class="button-grid">
                <div></div>
                <button class="btn-forward" onmousedown="sendVelocity(0.3, 0)" onmouseup="sendStop()">⬆️ Forward</button>
                <div></div>
                <button class="btn-left" onmousedown="sendVelocity(0, 0.5)" onmouseup="sendStop()">⬅️ Left</button>
                <button class="btn-stop" onclick="sendStop()">⏹️ STOP</button>
                <button class="btn-right" onmousedown="sendVelocity(0, -0.5)" onmouseup="sendStop()">➡️ Right</button>
                <div></div>
                <button class="btn-backward" onmousedown="sendVelocity(-0.3, 0)" onmouseup="sendStop()">⬇️ Backward</button>
                <div></div>
            </div>
        </div>
        
        <!-- Actuator Control -->
        <div class="control-section">
            <h2>⚡ Actuator Control</h2>
            <button id="relay-vacuum" class="btn-relay" onclick="toggleRelay('vacuum')">💨 Vacuum: OFF</button>
            <button id="relay-brush_main" class="btn-relay" onclick="toggleRelay('brush_main')">🔄 Main Brush: OFF</button>
            <button id="relay-brush_left" class="btn-relay" onclick="toggleRelay('brush_left')">◀️ Left Brush: OFF</button>
            <button id="relay-brush_right" class="btn-relay" onclick="toggleRelay('brush_right')">▶️ Right Brush: OFF</button>
        </div>
        
        <!-- LED & Buzzer Control -->
        <div class="control-section">
            <h2>🔊 Indicators</h2>
            <button id="aux-buzzer" class="btn-relay" onclick="toggleAux('BZ')">🔔 Buzzer: OFF</button>
            <button id="aux-led" class="btn-relay" onclick="toggleAux('LED')">💡 LED: OFF</button>
        </div>
        
        <div class="grid">
            <!-- Safety Status -->
            <div class="card">
                <h2>🛡️ Safety Status</h2>
                <div class="sensor-value">
                    <span class="sensor-label">Status:</span>
                    <span id="safety-status" class="sensor-data status-safe">SAFE</span>
                </div>
                <div class="sensor-value">
                    <span class="sensor-label">Pin 22 (Emergency):</span>
                    <span id="safety-pin22" class="sensor-data">1</span>
                </div>
                <div class="sensor-value">
                    <span class="sensor-label">Pin 24 (Bumper):</span>
                    <span id="safety-pin24" class="sensor-data">0</span>
                </div>
            </div>
            
            <!-- Ultrasonic Sensors -->
            <div class="card">
                <h2>📡 Ultrasonic Sensors (cm)</h2>
                <div class="sensor-value">
                    <span class="sensor-label">Front:</span>
                    <span id="us-front" class="sensor-data">0</span>
                </div>
                <div class="sensor-value">
                    <span class="sensor-label">Front Right:</span>
                    <span id="us-front-right" class="sensor-data">0</span>
                </div>
                <div class="sensor-value">
                    <span class="sensor-label">Front Left:</span>
                    <span id="us-front-left" class="sensor-data">0</span>
                </div>
                <div class="sensor-value">
                    <span class="sensor-label">Right:</span>
                    <span id="us-right" class="sensor-data">0</span>
                </div>
                <div class="sensor-value">
                    <span class="sensor-label">Left:</span>
                    <span id="us-left" class="sensor-data">0</span>
                </div>
            </div>
            
            <!-- Water Tanks -->
            <div class="card">
                <h2>💧 Water Tanks (cm)</h2>
                <div class="sensor-value">
                    <span class="sensor-label">Clean Tank:</span>
                    <span id="tank-clean" class="sensor-data">0</span>
                </div>
                <div class="sensor-value">
                    <span class="sensor-label">Dirty Tank:</span>
                    <span id="tank-dirty" class="sensor-data">0</span>
                </div>
            </div>
            
            <!-- IR Object Sensors -->
            <div class="card">
                <h2>👁️ IR Object Detection</h2>
                <div class="sensor-value">
                    <span class="sensor-label">Front Right:</span>
                    <span id="ir-obj-fr" class="sensor-data">0<span class="ir-indicator ir-clear"></span></span>
                </div>
                <div class="sensor-value">
                    <span class="sensor-label">Front Left:</span>
                    <span id="ir-obj-fl" class="sensor-data">0<span class="ir-indicator ir-clear"></span></span>
                </div>
                <div class="sensor-value">
                    <span class="sensor-label">Back Right:</span>
                    <span id="ir-obj-br" class="sensor-data">0<span class="ir-indicator ir-clear"></span></span>
                </div>
                <div class="sensor-value">
                    <span class="sensor-label">Back Left:</span>
                    <span id="ir-obj-bl" class="sensor-data">0<span class="ir-indicator ir-clear"></span></span>
                </div>
            </div>
            
            <!-- IR Stair Sensors -->
            <div class="card">
                <h2>🪜 IR Stair Detection</h2>
                <div class="sensor-value">
                    <span class="sensor-label">Front Right:</span>
                    <span id="ir-stair-fr" class="sensor-data">0<span class="ir-indicator ir-clear"></span></span>
                </div>
                <div class="sensor-value">
                    <span class="sensor-label">Front Left:</span>
                    <span id="ir-stair-fl" class="sensor-data">0<span class="ir-indicator ir-clear"></span></span>
                </div>
                <div class="sensor-value">
                    <span class="sensor-label">Back Right:</span>
                    <span id="ir-stair-br" class="sensor-data">0<span class="ir-indicator ir-clear"></span></span>
                </div>
                <div class="sensor-value">
                    <span class="sensor-label">Back Left:</span>
                    <span id="ir-stair-bl" class="sensor-data">0<span class="ir-indicator ir-clear"></span></span>
                </div>
            </div>
            
            <!-- Encoders -->
            <div class="card">
                <h2>⚙️ Wheel Encoders</h2>
                <div class="sensor-value">
                    <span class="sensor-label">Left Wheel:</span>
                    <span id="enc-left" class="sensor-data">0</span>
                </div>
                <div class="sensor-value">
                    <span class="sensor-label">Right Wheel:</span>
                    <span id="enc-right" class="sensor-data">0</span>
                </div>
            </div>
            
            <!-- Battery -->
            <div class="card">
                <h2>🔋 Battery Status</h2>
                <div class="sensor-value">
                    <span class="sensor-label">Voltage (raw):</span>
                    <span id="bat-voltage" class="sensor-data">0</span>
                </div>
                <div class="sensor-value">
                    <span class="sensor-label">Current (raw):</span>
                    <span id="bat-current" class="sensor-data">0</span>
                </div>
                <div class="sensor-value">
                    <span class="sensor-label">Temperature (raw):</span>
                    <span id="bat-temp" class="sensor-data">0</span>
                </div>
            </div>
            
            <!-- IMU -->
            <div class="card">
                <h2>🧭 IMU (BNO055)</h2>
                <div class="sensor-value">
                    <span class="sensor-label">Orientation (Quat):</span>
                    <span id="imu-quat" class="sensor-data">0, 0, 0, 0</span>
                </div>
                <div class="sensor-value">
                    <span class="sensor-label">Gyro (rad/s):</span>
                    <span id="imu-gyro" class="sensor-data">0, 0, 0</span>
                </div>
                <div class="sensor-value">
                    <span class="sensor-label">Magnetometer (µT):</span>
                    <span id="imu-mag" class="sensor-data">0, 0, 0</span>
                </div>
            </div>
            
            <!-- LiDAR -->
            <div class="card">
                <h2>🔴 LiDAR Status</h2>
                <div class="sensor-value">
                    <span class="sensor-label">Min Range:</span>
                    <span id="lidar-min" class="sensor-data">0</span>
                </div>
                <div class="sensor-value">
                    <span class="sensor-label">Max Range:</span>
                    <span id="lidar-max" class="sensor-data">0</span>
                </div>
                <div class="sensor-value">
                    <span class="sensor-label">Data Points:</span>
                    <span id="lidar-points" class="sensor-data">0</span>
                </div>
            </div>
        </div>
        
        <div class="update-time">Last Update: <span id="last-update">Never</span></div>
    </div>
    
    <script>
        let relayStates = {vacuum: false, brush_main: false, brush_left: false, brush_right: false};
        let auxStates = {BZ: false, LED: false};
        
        function sendVelocity(linear, angular) {
            fetch('/api/velocity', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({linear_x: linear, angular_z: angular})
            });
        }
        
        function sendStop() {
            sendVelocity(0, 0);
        }
        
        function toggleRelay(relay) {
            relayStates[relay] = !relayStates[relay];
            fetch('/api/relay', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({relay: relay, state: relayStates[relay]})
            });
            updateRelayButton(relay);
        }
        
        function toggleAux(device) {
            auxStates[device] = !auxStates[device];
            fetch('/api/aux', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({device: device, state: auxStates[device]})
            });
            updateAuxButton(device);
        }
        
        function updateRelayButton(relay) {
            const btn = document.getElementById(`relay-${relay}`);
            const name = relay.replace('_', ' ').replace(/\b\w/g, l => l.toUpperCase());
            btn.textContent = `${getRelayIcon(relay)} ${name}: ${relayStates[relay] ? 'ON' : 'OFF'}`;
            btn.className = relayStates[relay] ? 'btn-relay active' : 'btn-relay';
        }
        
        function updateAuxButton(device) {
            const btn = document.getElementById(`aux-${device.toLowerCase()}`);
            const names = {BZ: 'Buzzer', LED: 'LED'};
            const icons = {BZ: '🔔', LED: '💡'};
            btn.textContent = `${icons[device]} ${names[device]}: ${auxStates[device] ? 'ON' : 'OFF'}`;
            btn.className = auxStates[device] ? 'btn-relay active' : 'btn-relay';
        }
        
        function getRelayIcon(relay) {
            const icons = {vacuum: '💨', brush_main: '🔄', brush_left: '◀️', brush_right: '▶️'};
            return icons[relay] || '⚡';
        }
        
        function updateDashboard() {
            fetch('/api/data')
                .then(r => r.json())
                .then(data => {
                    // Safety
                    document.getElementById('safety-status').textContent = data.safety.status;
                    document.getElementById('safety-status').className = data.safety.safety_active ? 'sensor-data status-emergency' : 'sensor-data status-safe';
                    document.getElementById('safety-pin22').textContent = data.safety.pin22 || 'N/A';
                    document.getElementById('safety-pin24').textContent = data.safety.pin24 || 'N/A';
                    
                    // Ultrasonic
                    document.getElementById('us-front').textContent = data.ultrasonic.front.toFixed(1);
                    document.getElementById('us-front-right').textContent = data.ultrasonic.front_right.toFixed(1);
                    document.getElementById('us-front-left').textContent = data.ultrasonic.front_left.toFixed(1);
                    document.getElementById('us-right').textContent = data.ultrasonic.right.toFixed(1);
                    document.getElementById('us-left').textContent = data.ultrasonic.left.toFixed(1);
                    
                    // Water tanks
                    document.getElementById('tank-clean').textContent = data.water_tanks.clean.toFixed(1);
                    document.getElementById('tank-dirty').textContent = data.water_tanks.dirty.toFixed(1);
                    
                    // IR Object
                    updateIR('ir-obj-fr', data.ir_object.front_right);
                    updateIR('ir-obj-fl', data.ir_object.front_left);
                    updateIR('ir-obj-br', data.ir_object.back_right);
                    updateIR('ir-obj-bl', data.ir_object.back_left);
                    
                    // IR Stair
                    updateIR('ir-stair-fr', data.ir_stair.front_right);
                    updateIR('ir-stair-fl', data.ir_stair.front_left);
                    updateIR('ir-stair-br', data.ir_stair.back_right);
                    updateIR('ir-stair-bl', data.ir_stair.back_left);
                    
                    // Encoders
                    document.getElementById('enc-left').textContent = data.encoders.left;
                    document.getElementById('enc-right').textContent = data.encoders.right;
                    
                    // Battery
                    document.getElementById('bat-voltage').textContent = data.battery.voltage;
                    document.getElementById('bat-current').textContent = data.battery.current;
                    document.getElementById('bat-temp').textContent = data.battery.temperature;
                    
                    // IMU
                    document.getElementById('imu-quat').textContent = `${data.imu.qw.toFixed(2)}, ${data.imu.qx.toFixed(2)}, ${data.imu.qy.toFixed(2)}, ${data.imu.qz.toFixed(2)}`;
                    document.getElementById('imu-gyro').textContent = `${data.imu.gx.toFixed(2)}, ${data.imu.gy.toFixed(2)}, ${data.imu.gz.toFixed(2)}`;
                    document.getElementById('imu-mag').textContent = `${data.imu.mx.toFixed(2)}, ${data.imu.my.toFixed(2)}, ${data.imu.mz.toFixed(2)}`;
                    
                    // LiDAR
                    document.getElementById('lidar-min').textContent = data.lidar.min_range.toFixed(2);
                    document.getElementById('lidar-max').textContent = data.lidar.max_range.toFixed(2);
                    document.getElementById('lidar-points').textContent = data.lidar.ranges.length;
                    
                    // Update relay states from server
                    relayStates = data.relays;
                    Object.keys(relayStates).forEach(updateRelayButton);
                    
                    // Last update time
                    document.getElementById('last-update').textContent = new Date().toLocaleTimeString();
                });
        }
        
        function updateIR(id, value) {
            const elem = document.getElementById(id);
            const indicator = elem.querySelector('.ir-indicator');
            elem.childNodes[0].textContent = value + ' ';
            indicator.className = value === 0 ? 'ir-indicator ir-blocked' : 'ir-indicator ir-clear';
        }
        
        // Update every 200ms
        setInterval(updateDashboard, 200);
        updateDashboard();
    </script>
</body>
</html>
"""

@app.route('/')
def index():
    return render_template_string(HTML_TEMPLATE)

@app.route('/api/data')
def get_data():
    if dashboard_node:
        return jsonify(dashboard_node.sensor_data)
    return jsonify({})

@app.route('/api/velocity', methods=['POST'])
def set_velocity():
    if dashboard_node:
        data = request.json
        dashboard_node.send_velocity(data['linear_x'], data['angular_z'])
        return jsonify({'status': 'ok'})
    return jsonify({'status': 'error'})

@app.route('/api/relay', methods=['POST'])
def set_relay():
    if dashboard_node:
        data = request.json
        dashboard_node.send_relay_command(data['relay'], data['state'])
        return jsonify({'status': 'ok'})
    return jsonify({'status': 'error'})

@app.route('/api/aux', methods=['POST'])
def set_aux():
    if dashboard_node:
        data = request.json
        dashboard_node.send_aux_command(data['device'], data['state'])
        return jsonify({'status': 'ok'})
    return jsonify({'status': 'error'})

def run_flask():
    app.run(host='0.0.0.0', port=5000, debug=False, use_reloader=False)

def main():
    global dashboard_node
    rclpy.init()
    dashboard_node = RobotDashboard()
    
    # Start Flask in separate thread
    flask_thread = threading.Thread(target=run_flask, daemon=True)
    flask_thread.start()
    
    print("=" * 60)
    print("🤖 Robot Dashboard Started!")
    print("=" * 60)
    print("Open your browser to: http://localhost:5000")
    print("Or from another device: http://<robot-ip>:5000")
    print("=" * 60)
    
    try:
        rclpy.spin(dashboard_node)
    except KeyboardInterrupt:
        pass
    finally:
        dashboard_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
