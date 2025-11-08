# 🤖 Autonomous Floor Cleaning Robot - Complete System

## Package: `robot_sensors` (formerly ultrasonic_bridge)

### Why the Name Change?
The package now handles **ALL robot sensors**, not just ultrasonic:
- ✅ 5x Ultrasonic sensors
- ✅ 8x IR sensors (object + stair detection)
- ✅ 2x Wheel encoders
- ✅ Motor driver interface
- ✅ LIDAR integration

**`robot_sensors`** better reflects the complete sensor suite! 🎯

## Complete Sensor Array

### Total: 20 I/O + LIDAR
```
┌─────────────────────────────────────────┐
│     Autonomous Floor Cleaning Robot    │
├─────────────────────────────────────────┤
│ • 5x HC-SR04 Ultrasonic (distance)      │
│ • 4x IR Object Detection                │
│ • 4x IR Stair Detection (safety)        │
│ • 2x Wheel Encoders (odometry)          │
│ • 1x SLLIDAR A1 (360° mapping)          │
│ • 4x Motor PWM (BTS7960 driver)         │
└─────────────────────────────────────────┘
```

## 🚀 Quick Start

### 1. Upload Multi-Sensor Arduino Code
```bash
cd ~/clean_ws/src/robot_sensors/arduino/multi_sensor
pio run --target upload
```

### 2. Launch Complete Robot System
```bash
cd ~/clean_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch robot_sensors robot_launch.py
```

This starts:
- ✅ SLLIDAR node (`/scan`)
- ✅ Multi-sensor bridge (all Arduino sensors)

### 3. Monitor Sensor Data
```bash
# List all topics
ros2 topic list

# View raw sensor data
ros2 topic echo /sensors/raw

# Individual sensors
ros2 topic echo /ultrasonic/front
ros2 topic echo /ir/front_right/object
ros2 topic echo /ir/front_right/stair
ros2 topic echo /encoder/left
ros2 topic echo /scan
```

## 📊 ROS Topics Published (19 total)

### Ultrasonic Range (5)
- `/ultrasonic/front`
- `/ultrasonic/front_right`
- `/ultrasonic/front_left`
- `/ultrasonic/right`
- `/ultrasonic/left`

### IR Object Detection (4)
- `/ir/front_right/object`
- `/ir/front_left/object`
- `/ir/back_right/object`
- `/ir/back_left/object`

### IR Stair Detection (4)
- `/ir/front_right/stair`
- `/ir/front_left/stair`
- `/ir/back_right/stair`
- `/ir/back_left/stair`

### Encoders (2)
- `/encoder/left`
- `/encoder/right`

### LIDAR (1)
- `/scan`

### Debug (3)
- `/sensors/raw`
- `/parameter_events`
- `/rosout`

## 📁 Project Structure

```
~/clean_ws/
├── src/
│   ├── robot_sensors/           ← Main sensor package
│   │   ├── arduino/
│   │   │   ├── hcsr04_serial/   (legacy - single sensor)
│   │   │   └── multi_sensor/    ← Current - all sensors
│   │   │       ├── platformio.ini
│   │   │       └── src/main.cpp
│   │   ├── launch/
│   │   │   ├── robot_launch.py  ← Main launch file
│   │   │   └── sensors_launch.py
│   │   ├── robot_sensors/       ← Python package
│   │   │   ├── multi_sensor_node.py  ← Main node
│   │   │   └── serial_range_node.py  (legacy)
│   │   ├── scripts/
│   │   │   ├── test_all_sensors.sh
│   │   │   └── test_bridge.sh
│   │   ├── package.xml
│   │   ├── setup.py
│   │   └── SENSORS_README.md
│   │
│   └── sllidar_ros2/            ← LIDAR package
│
├── install/
├── build/
└── ROBOT_STATUS.md              ← This file
```

## 🔧 Hardware Connections

See `src/robot_sensors/SENSORS_README.md` for complete wiring diagram.

## 🦊 Foxglove Visualization

```bash
# Terminal 1: Robot sensors
ros2 launch robot_sensors robot_launch.py

# Terminal 2: Foxglove bridge
source /opt/ros/humble/setup.bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765
```

Connect Foxglove Studio to: `ws://localhost:8765`

## ✅ System Status

| Component | Status | Details |
|-----------|--------|---------|
| Arduino Code | ✅ Uploaded | Multi-sensor firmware v1.0 |
| ROS Package | ✅ Built | robot_sensors v0.1.0 |
| LIDAR | ✅ Working | SLLIDAR A1 @ /dev/ttyUSB0 |
| Multi-Sensor | ✅ Working | Arduino Mega @ /dev/ttyACM0 |
| Launch Files | ✅ Ready | robot_launch.py |
| Documentation | ✅ Complete | SENSORS_README.md |

## 🎯 Ready for Autonomous Navigation! 🧹✨
