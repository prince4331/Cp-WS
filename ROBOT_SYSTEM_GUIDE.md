# 🤖 Autonomous Floor Cleaning Robot - Complete System Guide

**Last Updated:** November 2025  
**ROS2 Version:** Humble  
**Hardware:** Raspberry Pi 4B + Arduino Mega + SLLIDAR A1M8

---

## 📋 Table of Contents
1. [Quick Start](#quick-start)
2. [Hardware Setup](#hardware-setup)
3. [System Architecture](#system-architecture)
4. [Available Launch Scripts](#available-launch-scripts)
5. [Manual Control](#manual-control)
6. [Troubleshooting](#troubleshooting)
7. [Development](#development)

---

## 🚀 Quick Start

### Start Robot for Manual Control (Recommended)
```bash
cd /home/saad/clean_ws
./start_industry_grade.sh
```

Then open dashboard: **http://192.168.0.181:3000**

### Stop All Systems
```bash
pkill -f "ros2|python3.*robot_sensors"
```

---

## 🔧 Hardware Setup

### Complete Sensor Array
```
┌─────────────────────────────────────────┐
│     Autonomous Floor Cleaning Robot    │
├─────────────────────────────────────────┤
│ • 5x HC-SR04 Ultrasonic (distance)      │
│ • 4x IR Object Detection                │
│ • 4x IR Stair Detection (safety)        │
│ • 2x Wheel Encoders (odometry)          │
│ • 1x SLLIDAR A1M8 (360° mapping)        │
│ • 2x BTS7960 Motor Drivers              │
│ • 1x Arduino Mega 2560                  │
│ • 1x Raspberry Pi 4B (4GB)              │
└─────────────────────────────────────────┘
```

### Pin Connections

#### Arduino Mega Connections
**Ultrasonic Sensors (HC-SR04):**
- Front: Trigger=22, Echo=23
- Front-Left: Trigger=24, Echo=25
- Front-Right: Trigger=26, Echo=27
- Left: Trigger=28, Echo=29
- Right: Trigger=30, Echo=31

**IR Object Detection:**
- Front: Pin 32
- Front-Left: Pin 33
- Front-Right: Pin 34
- Rear: Pin 35

**IR Stair Detection:**
- Front-Left: Pin 36
- Front-Right: Pin 37
- Rear-Left: Pin 38
- Rear-Right: Pin 39

**Wheel Encoders:**
- Left: Pin 2 (interrupt)
- Right: Pin 3 (interrupt)

**Motor Control (BTS7960):**
- Left Motor: LPWM=4, RPWM=5
- Right Motor: LPWM=6, RPWM=7
- Enable Pins: L_EN=8, R_EN=9

**Communication:**
- Serial: 115200 baud (USB to Raspberry Pi)
- Protocol: Custom binary protocol (0.05s timeout)

#### Raspberry Pi Connections
- **Arduino:** USB connection (/dev/ttyACM0)
- **SLLIDAR A1M8:** USB connection (/dev/ttyUSB0, 115200 baud)
- **Network:** WiFi (192.168.0.181)

---

## 🏗️ System Architecture

### ROS2 Nodes

#### Core Nodes (Always Running)
1. **multi_sensor_node** - Arduino sensor bridge
   - Publishes: `/ultrasonic/*`, `/ir/*`, `/encoder/*`, `/battery`, `/imu`
   - Subscribes: `/cmd_vel` (for motor control)
   - Serial: 115200 baud, 0.05s timeout
   
2. **motor_controller_node** - Motor interface
   - Subscribes: `/cmd_vel`
   - Publishes motor commands to Arduino
   - Safety timeout: 500ms

3. **sllidar_node** - LIDAR scanner
   - Publishes: `/scan` (LaserScan)
   - Frame: `laser`
   - Scan frequency: 10Hz

4. **odometry_node** - Wheel odometry
   - Subscribes: `/encoder/left`, `/encoder/right`
   - Publishes: `/odom` (Odometry)
   - Publishes TF: `odom` → `base_link`

5. **rosbridge_websocket** - Web interface bridge
   - Port: 9090
   - Protocol: WebSocket
   - Used by dashboard

#### Optional Nodes
6. **industry_obstacle_avoidance** - Autonomous obstacle detection
   - Only for autonomous mode
   - Disable for manual control: `pkill -f industry_obstacle_avoidance`

7. **slam_toolbox** - Real-time mapping
   - Requires: `base_frame:=base_link`
   - Only needed for mapping

### TF Tree
```
odom (world frame)
 └─ base_link (robot center)
     └─ laser (LIDAR sensor, +0.15m Z-axis)
```

### Topics
**Sensor Data:**
- `/scan` - LIDAR scan data (LaserScan)
- `/ultrasonic/front` - Front ultrasonic (Range)
- `/ultrasonic/front_left` - Front-left ultrasonic (Range)
- `/ultrasonic/front_right` - Front-right ultrasonic (Range)
- `/ultrasonic/left` - Left ultrasonic (Range)
- `/ultrasonic/right` - Right ultrasonic (Range)
- `/ir/object/front` - Front IR object (Bool)
- `/ir/object/front_left` - Front-left IR object (Bool)
- `/ir/object/front_right` - Front-right IR object (Bool)
- `/ir/object/rear` - Rear IR object (Bool)
- `/ir/stair/front_left` - Front-left stair (Bool)
- `/ir/stair/front_right` - Front-right stair (Bool)
- `/ir/stair/rear_left` - Rear-left stair (Bool)
- `/ir/stair/rear_right` - Rear-right stair (Bool)
- `/battery` - Battery voltage (Float32)
- `/imu` - IMU data (Imu)
- `/encoder/left` - Left encoder (Int32)
- `/encoder/right` - Right encoder (Int32)
- `/odom` - Odometry (Odometry)

**Control:**
- `/cmd_vel` - Velocity commands (Twist)

---

## 📜 Available Launch Scripts

### 1. `start_industry_grade.sh` ⭐ **RECOMMENDED**
**Best for:** Manual control with dashboard
```bash
./start_industry_grade.sh
```
**Starts:**
- Multi-sensor bridge
- SLLIDAR
- Motor controller
- Odometry
- Industry obstacle avoidance (disable for manual: `pkill -f industry_obstacle_avoidance`)
- Rosbridge WebSocket

**Dashboard:** http://192.168.0.181:3000

### 2. `create_map_with_keyboard.sh`
**Best for:** Creating maps with keyboard control
```bash
./create_map_with_keyboard.sh
```
**Features:**
- SLAM mapping
- Keyboard teleop (WASD)
- Auto-saves map on exit

### 3. `start_robot_system.sh`
**Best for:** Basic robot startup (no autonomous features)
```bash
./start_robot_system.sh
```

---

## 🎮 Manual Control

### Web Dashboard (Recommended)

1. **Start the system:**
   ```bash
   cd /home/saad/clean_ws
   ./start_industry_grade.sh
   ```

2. **Disable obstacle avoidance (for manual control):**
   ```bash
   pkill -f industry_obstacle_avoidance
   ```

3. **Start dashboard:**
   ```bash
   cd /home/saad/clean_ws/robot-dashboard
   npm run dev -- --host 192.168.0.181 --port 3000
   ```

4. **Open in browser:**
   - Computer: http://192.168.0.181:3000
   - Phone/Tablet: http://192.168.0.181:3000

5. **Control methods:**
   - Virtual joystick (touch/mouse)
   - Keyboard: WASD or Arrow keys
   - Commands publish at 10Hz for smooth movement

### Direct ROS2 Control

**Publish velocity commands:**
```bash
# Move forward
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.2}}" --once

# Rotate
ros2 topic pub /cmd_vel geometry_msgs/Twist "{angular: {z: 0.5}}" --once

# Stop
ros2 topic pub /cmd_vel geometry_msgs/Twist "{}" --once
```

---

## 🔍 Troubleshooting

### Dashboard Shows "No Data"

**Symptom:** Dashboard displays "totally 0" or waiting for data

**Solution:**
```bash
# 1. Check multi_sensor node is running
ros2 node list | grep multi_sensor

# 2. Check serial connection
ls -l /dev/ttyACM0

# 3. Check data publishing
ros2 topic hz /ultrasonic/front

# 4. Restart multi_sensor node
pkill -f multi_sensor_node
ros2 run robot_sensors multi_sensor_node
```

**Root Cause:** Serial timeout too long (was 1.0s, fixed to 0.05s in code)

### Robot Not Moving

**Symptom:** Publishing to `/cmd_vel` but robot doesn't move

**Solutions:**
```bash
# 1. Check obstacle avoidance is not blocking
pkill -f obstacle_avoidance

# 2. Check motor controller is running
ros2 node list | grep motor_controller

# 3. Check Arduino connection
ls -l /dev/ttyACM0

# 4. Verify commands received
ros2 topic echo /cmd_vel
```

### Jerky Robot Movement

**Symptom:** Robot moves in stop-start pattern

**Root Cause:** Motor controller has 500ms safety timeout. Commands must be sent at >2Hz.

**Solution:** Use dashboard (publishes at 10Hz) or keyboard control with continuous publishing.

### SLAM Not Working

**Symptom:** "Failed to compute odom pose" or "Message Filter dropping message"

**Solutions:**
```bash
# 1. Check TF tree
ros2 run tf2_ros tf2_echo odom laser

# 2. Set correct base_frame
ros2 launch slam_toolbox online_async_launch.py base_frame:=base_link

# 3. Verify odometry publishing TF
ros2 topic echo /odom --once

# 4. Check laser frame_id
ros2 topic echo /scan --once | grep frame_id
```

**Note:** SLAM requires robot movement to build initial map. Map won't appear until robot moves.

### Rosbridge Connection Issues

**Symptom:** Dashboard can't connect to robot

**Solutions:**
```bash
# 1. Check rosbridge running
netstat -tuln | grep 9090

# 2. Kill duplicate instances
pkill -f rosbridge
sleep 2

# 3. Restart rosbridge
ros2 launch rosbridge_server rosbridge_websocket_launch.xml

# 4. Check firewall
sudo ufw allow 9090
```

### LIDAR Not Detected

**Symptom:** `/scan` topic not publishing

**Solutions:**
```bash
# 1. Check USB connection
ls -l /dev/ttyUSB0

# 2. Check permissions
sudo chmod 666 /dev/ttyUSB0

# 3. Add to dialout group (permanent fix)
sudo usermod -a -G dialout $USER
# Then logout/login

# 4. Check LIDAR spinning
# Green LED should blink when spinning

# 5. Restart LIDAR node
pkill -f sllidar
ros2 launch sllidar_ros2 sllidar_a1_launch.py
```

---

## 💻 Development

### Project Structure
```
/home/saad/clean_ws/
├── src/
│   └── robot_sensors/          # Main ROS2 package
│       ├── robot_sensors/      # Python nodes
│       │   ├── multi_sensor_node.py        # Arduino bridge
│       │   ├── motor_controller_node.py    # Motor interface
│       │   ├── odometry_node.py            # Wheel odometry
│       │   └── industry_obstacle_avoidance.py  # Obstacle detection
│       ├── arduino/
│       │   └── multi_sensor/   # PlatformIO Arduino code
│       ├── package.xml
│       └── setup.py
├── robot-dashboard/            # React/TypeScript dashboard
│   ├── src/
│   │   ├── components/
│   │   │   └── widgets/
│   │   │       └── Joystick.tsx  # Virtual joystick
│   │   ├── pages/
│   │   │   └── ManualControl.tsx # Keyboard control
│   │   └── lib/
│   │       └── rosbridge.ts      # ROS WebSocket
│   ├── package.json
│   └── vite.config.ts
├── start_industry_grade.sh     # ⭐ Main startup script
├── create_map_with_keyboard.sh # Mapping script
└── ROBOT_SYSTEM_GUIDE.md       # This file
```

### Building the Workspace

```bash
cd /home/saad/clean_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select robot_sensors
source install/setup.bash
```

### Modifying Arduino Code

```bash
cd /home/saad/clean_ws/src/robot_sensors/arduino/multi_sensor
# Edit src/main.cpp
pio run --target upload
```

### Modifying ROS2 Nodes

Python nodes are installed in editable mode. Changes take effect immediately:
```bash
# Edit file
nano /home/saad/clean_ws/src/robot_sensors/robot_sensors/multi_sensor_node.py

# Restart node
pkill -f multi_sensor_node
ros2 run robot_sensors multi_sensor_node
```

### Dashboard Development

```bash
cd /home/saad/clean_ws/robot-dashboard
npm run dev -- --host 192.168.0.181 --port 3000
# Changes hot-reload automatically
```

---

## 📊 System Status

### Check All Nodes
```bash
ros2 node list
```

Expected output:
```
/motor_controller
/multi_sensor_bridge
/odometry_node
/rosbridge_websocket
/sllidar_node
```

### Check All Topics
```bash
ros2 topic list
```

### Monitor Sensor Data
```bash
# LIDAR
ros2 topic hz /scan

# Ultrasonic
ros2 topic echo /ultrasonic/front

# Odometry
ros2 topic echo /odom

# All at once
ros2 topic list | grep -E "scan|ultrasonic|odom|cmd_vel"
```

### System Logs
```bash
# Multi-sensor
tail -f /tmp/multi_sensor.log

# Motor controller
tail -f /tmp/motor.log

# LIDAR
tail -f /tmp/lidar.log

# Odometry
tail -f /tmp/odometry.log

# Rosbridge
tail -f /tmp/rosbridge.log
```

---

## 🔐 Permissions

### USB Device Access
```bash
# Temporary (until reboot)
sudo chmod 666 /dev/ttyACM0  # Arduino
sudo chmod 666 /dev/ttyUSB0  # LIDAR

# Permanent (add user to dialout group)
sudo usermod -a -G dialout $USER
# Then logout/login
```

---

## 📝 Important Notes

### Serial Communication
- **Timeout:** 0.05s (50ms) for fast response
- **Rate:** ~8Hz for sensor data
- **Protocol:** Custom binary format for efficiency

### Motor Safety
- **Timeout:** 500ms without commands = STOP
- **Solution:** Publish commands at ≥2Hz (dashboard uses 10Hz)

### Frame Conventions
- **odom:** World frame (fixed)
- **base_link:** Robot center (moves with robot)
- **laser:** LIDAR position (0.15m above base_link)

### Network
- **Robot IP:** 192.168.0.181
- **Dashboard Port:** 3000
- **Rosbridge Port:** 9090

---

## 🆘 Support

### Quick Diagnostics
```bash
# Check everything at once
ros2 node list && ros2 topic list && netstat -tuln | grep -E "3000|9090"
```

### Full System Restart
```bash
# Stop everything
pkill -f "ros2|python3.*robot_sensors|npm"
sleep 2

# Restart
cd /home/saad/clean_ws
./start_industry_grade.sh

# Open dashboard
cd robot-dashboard
npm run dev -- --host 192.168.0.181 --port 3000
```

---

## 📚 Resources

- **ROS2 Humble Docs:** https://docs.ros.org/en/humble/
- **SLLIDAR SDK:** https://github.com/Slamtec/sllidar_ros2
- **Rosbridge:** https://github.com/RobotWebTools/rosbridge_suite
- **SLAM Toolbox:** https://github.com/SteveMacenski/slam_toolbox

---

**Last Updated:** November 2025  
**Maintainer:** Robot Development Team  
**Version:** 2.0 (Industry Grade)
