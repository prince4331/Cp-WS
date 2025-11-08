# Autonomous Floor Cleaning Robot - Sensor Configuration

## Hardware Setup (Arduino Mega 2560)

### Ultrasonic Sensors (HC-SR04) - 7 Units
| Sensor | Trigger Pin | Echo Pin | ROS Topic |
|--------|-------------|----------|-----------|
| Front | 44 | 45 | `/ultrasonic/front` |
| Front Right | 42 | 43 | `/ultrasonic/front_right` |
| Front Left | 40 | 41 | `/ultrasonic/front_left` |
| Right | 38 | 39 | `/ultrasonic/right` |
| Left | 36 | 37 | `/ultrasonic/left` |
| Clean Water Tank | 34 | 35 | `/water_level/clean` |
| Dirty Water Tank | 32 | 33 | `/water_level/dirty` |

### IR Sensors - Object Detection (4 Units)
| Sensor | Pin | ROS Topic |
|--------|-----|-----------|
| Front Right | 46 | `/ir/front_right/object` |
| Front Left | 48 | `/ir/front_left/object` |
| Back Right | 50 | `/ir/back_right/object` |
| Back Left | 52 | `/ir/back_left/object` |

### IR Sensors - Stair Detection (4 Units)
| Sensor | Pin | ROS Topic |
|--------|-----|-----------|
| Front Right | 47 | `/ir/front_right/stair` |
| Front Left | 49 | `/ir/front_left/stair` |
| Back Right | 51 | `/ir/back_right/stair` |
| Back Left | 53 | `/ir/back_left/stair` |

### Wheel Encoders (2 Units)
| Encoder | Pin | ROS Topic |
|---------|-----|-----------|
| Left | 3 (Interrupt) | `/encoder/left` |
| Right | 2 (Interrupt) | `/encoder/right` |

### Emergency / Safety Switches
| Device | Pin | ROS Topic |
|--------|-----|-----------|
| Main Emergency Stop (NO) | 24 | `/safety/estop_main` |
| Front Bumper / Pressure Switch (NC) | 22 | `/safety/front_switch` |

### Auxiliary Outputs
| Device | Pin | ROS Topic |
|--------|-----|-----------|
| Buzzer | 26 | `/buzzer/state`, command `/buzzer/cmd` |
| Status LED | 28 | `/indicator/state`, command `/indicator/cmd` |
| Vacuum Motor Relay | 30 | `/vacuum/state`, command `/vacuum/cmd` |
| Main Brush Relay | 31 | `/brush/main/state`, command `/brush/main/cmd` |
| Side Brush Relay (Left) | 27 | `/brush/left/state`, command `/brush/left/cmd` |
| Side Brush Relay (Right) | 29 | `/brush/right/state`, command `/brush/right/cmd` |

### Battery Telemetry (Analog)
| Measurement | Pin | Notes |
|-------------|-----|-------|
| Voltage Divider | A0 | Converted to `/battery/state` (BatteryState). Default scale assumes 19.1 kΩ / 6.2 kΩ divider (≈24 V pack). Override via `battery_voltage_scale` parameter if your resistors differ. |
| Current Sensor | A1 | Converted to `/battery/state` |
| Temperature Sensor | A2 | Converted to `/battery/state` |

### IMU
| Device | Bus | ROS Topic |
|--------|-----|-----------|
| Adafruit BNO055 (9-DoF) | I2C (0x28) | `/imu/data` |

### Motor Driver (BTS7960) - 4 PWM Outputs
| Motor | RPWM Pin | LPWM Pin |
|-------|----------|----------|
| Right | 6 | 7 |
| Left | 4 | 5 |

### LIDAR
| Device | Port | ROS Topic |
|--------|------|-----------|
| SLLIDAR A1 | /dev/ttyUSB0 | `/scan` |

## ROS 2 Topics Published

### Range Sensors (sensor_msgs/Range)
- `/ultrasonic/front`
- `/ultrasonic/front_right`
- `/ultrasonic/front_left`
- `/ultrasonic/right`
- `/ultrasonic/left`

### Water Level Sensors (sensor_msgs/Range)
- `/water_level/clean` - Distance from sensor to clean water surface
- `/water_level/dirty` - Distance from sensor to dirty water surface

### Boolean Sensors (std_msgs/Bool)
- `/ir/front_right/object` - True = object detected
- `/ir/front_left/object`
- `/ir/back_right/object`
- `/ir/back_left/object`
- `/ir/front_right/stair` - True = stair/drop detected
- `/ir/front_left/stair`
- `/ir/back_right/stair`
- `/ir/back_left/stair`
- `/safety/estop_main` - True when the hardware E-Stop is pressed
- `/safety/front_switch` - True when the front bumper switch is pressed
- `/safety/estop` - Aggregated safety flag (True if any emergency switch active)
- `/buzzer/state` - True when buzzer output is active
- `/indicator/state` - True when status LED output is active
- `/vacuum/state` - True when the vacuum relay is energized
- `/brush/main/state` - True when the main brush relay is energized
- `/brush/left/state` - True when the left side brush relay is energized
- `/brush/right/state` - True when the right side brush relay is energized

### Encoders (std_msgs/Int32)
- `/encoder/left` - Left wheel tick count
- `/encoder/right` - Right wheel tick count

### Battery (sensor_msgs/BatteryState)
- `/battery/state` - Voltage, current, and temperature converted from analog readings

### IMU (sensor_msgs/Imu)
- `/imu/data` - Quaternion orientation, angular velocity, and linear acceleration

### Auxiliary Commands (std_msgs/Bool)
- `/buzzer/cmd` - Set buzzer on/off (forwarded to Arduino `BZ:1/0`)
- `/indicator/cmd` - Set status LED on/off (forwarded to Arduino `LED:1/0`)
- `/vacuum/cmd` - Toggle vacuum relay (`VAC:1/0`)
- `/brush/main/cmd` - Toggle main brush relay (`BRM:1/0`)
- `/brush/left/cmd` - Toggle left side brush relay (`BRL:1/0`)
- `/brush/right/cmd` - Toggle right side brush relay (`BRR:1/0`)

### LIDAR (sensor_msgs/LaserScan)
- `/scan` - 360° laser scan data

### Debug
- `/sensors/raw` - Raw serial data from Arduino

## Quick Start

### 1. Upload Arduino Code
```bash
cd ~/clean_ws/src/robot_sensors/arduino/multi_sensor
pio run --target upload
```

### 2. Launch All Sensors
```bash
cd ~/clean_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch robot_sensors robot_launch.py
```

### 3. View All Topics
```bash
# In another terminal
source /opt/ros/humble/setup.bash
source ~/clean_ws/install/setup.bash
ros2 topic list
```

### 4. Monitor Specific Sensors
```bash
# Ultrasonic sensors
ros2 topic echo /ultrasonic/front

# IR object detection
ros2 topic echo /ir/front_right/object

# IR stair detection
ros2 topic echo /ir/front_right/stair

# Encoders
ros2 topic echo /encoder/left

# LIDAR
ros2 topic echo /scan
```

## Visualization with Foxglove

```bash
# Terminal 1: Launch sensors
ros2 launch ultrasonic_bridge robot_launch.py

# Terminal 2: Launch Foxglove Bridge
source /opt/ros/humble/setup.bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765
```

Then connect Foxglove Studio to `ws://localhost:8765`

## Data Format

Arduino outputs data as comma-separated values:
```
US:front,fright,fleft,right,left|USW:clean,dirty|IR_OBJ:fr,fl,br,bl|IR_STAIR:fr,fl,br,bl|ENC:left,right|ESTOP:main,front|BAT:volt_raw,curr_raw,temp_raw|IMU:qw,qx,qy,qz,gx,gy,gz,ax,ay,az|STATE:buzzer,led
```

Example:
```
US:25.43,30.12,28.67,45.21,50.34|USW:12.50,35.70|IR_OBJ:1,1,1,1|IR_STAIR:1,1,1,1|ENC:1523,1498|ESTOP:0,0|BAT:615,420,350|IMU:0.99,0.01,0.02,0.05,0.01,0.00,0.00,0.02,-0.01,9.81|STATE:0,1
```

If the IMU is not available, the section is reported as `IMU:NA`.

## Safety Features

- **Stair Detection**: 4 IR sensors detect floor drops/stairs to prevent falls
- **Object Detection**: 4 IR sensors detect small objects
- **Ultrasonic Array**: 5 sensors provide 270° obstacle detection
- **Water Level Monitoring**: Clean/dirty tank depth sensors for refill alerts
- **Emergency Stops**: Hardware E-stop and front bumper switch both reported to ROS
- **Battery Health**: Voltage/current/temperature telemetry for diagnostics
- **IMU Feedback**: 9-DoF IMU for attitude estimation
- **LIDAR**: 360° environment mapping

## Project Structure
```
src/robot_sensors/
├── arduino/
│   ├── hcsr04_serial/       # Single sensor (legacy)
│   └── multi_sensor/        # All sensors (current)
│       ├── platformio.ini
│       └── src/main.cpp
├── launch/
│   ├── robot_launch.py      # Main launch file
│   └── sensors_launch.py    # Legacy
├── scripts/
│   └── test_bridge.sh
└── robot_sensors/
    ├── multi_sensor_node.py # Multi-sensor bridge
    └── serial_range_node.py # Single sensor (legacy)
```

## Status: ✅ All Systems Ready
- Arduino Code: Uploaded
- ROS Package: Built
- Launch File: Configured
- Ready for autonomous navigation!
