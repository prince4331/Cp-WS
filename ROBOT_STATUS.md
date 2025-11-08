# Autonomous Floor Cleaning Robot - Complete System

## ✅ All Sensors Integrated and Working!

### Sensor Array Summary:
```
                    FRONT
              [US]    [US]    [US]
           FrontLeft Front FrontRight
              [IR]    [IR]    [IR]
           OBJ/STAIR      OBJ/STAIR

    LEFT                        RIGHT
    [US]                        [US]
    
              [ENCODER]  [ENCODER]
              Left Motor Right Motor
              [BTS7960]  [BTS7960]
              
                    BACK
              [IR]          [IR]
           BackLeft      BackRight
           OBJ/STAIR     OBJ/STAIR

        [SLLIDAR A1 - 360° LIDAR]
```

### Total Sensor Count:
- **5x** Ultrasonic Sensors (HC-SR04) - Distance measurement
- **4x** IR Object Sensors - Small object detection  
- **4x** IR Stair Sensors - Drop/stair detection (safety)
- **2x** Wheel Encoders - Odometry
- **1x** LIDAR (SLLIDAR A1) - 360° environment mapping
- **4x** Motor PWM Outputs (BTS7960 driver)

**Total: 20 I/O connections + Serial communication**

## Quick Commands

### Upload Arduino Code
```bash
cd ~/clean_ws/src/robot_sensors/arduino/multi_sensor
pio run --target upload
```

### Launch Everything
```bash
cd ~/clean_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

# Option 1: Launch all sensors (LIDAR + Multi-sensor)
ros2 launch robot_sensors robot_launch.py

# Option 2: Just multi-sensor bridge
ros2 run robot_sensors multi_sensor_node
```

### Monitor Topics
```bash
# List all topics
ros2 topic list

# Monitor raw sensor data
ros2 topic echo /sensors/raw

# Monitor specific sensors
ros2 topic echo /ultrasonic/front
ros2 topic echo /ir/front_right/object
ros2 topic echo /ir/front_right/stair
ros2 topic echo /encoder/left
```

### Foxglove Visualization
```bash
# Terminal 1: Launch robot
ros2 launch ultrasonic_bridge robot_launch.py

# Terminal 2: Foxglove bridge
source /opt/ros/humble/setup.bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765

# Open browser: https://app.foxglove.dev
# Connect to: ws://localhost:8765
```

## ROS Topics Published (19 total)

### Ultrasonic Range Sensors (5)
- `/ultrasonic/front` (sensor_msgs/Range)
- `/ultrasonic/front_right`
- `/ultrasonic/front_left`
- `/ultrasonic/right`
- `/ultrasonic/left`

### IR Object Detection (4)
- `/ir/front_right/object` (std_msgs/Bool)
- `/ir/front_left/object`
- `/ir/back_right/object`
- `/ir/back_left/object`

### IR Stair Detection (4)
- `/ir/front_right/stair` (std_msgs/Bool)
- `/ir/front_left/stair`
- `/ir/back_right/stair`
- `/ir/back_left/stair`

### Encoders (2)
- `/encoder/left` (std_msgs/Int32)
- `/encoder/right`

### LIDAR (1)
- `/scan` (sensor_msgs/LaserScan)

### Debug (1)
- `/sensors/raw` (std_msgs/String)

### System Topics (2)
- `/parameter_events`
- `/rosout`

## Hardware Wiring Reference

See `SENSORS_README.md` for complete pin mapping.

## Project Status: 🟢 READY FOR AUTONOMOUS NAVIGATION

All sensors tested and operational! 🤖🧹✨
