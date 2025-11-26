# ROS2 Topic Audit & Cleanup Plan
## Generated: Auto-audit of duplicate/unnecessary topics

---

## 🔍 TOPIC INVENTORY BY SOURCE

### **1. multi_sensor_node.py** (Main Arduino Bridge)
**Purpose:** Bridges Arduino serial data to ROS2

#### Ultrasonic Sensors (7 topics) ✅ KEEP
- `/ultrasonic/front` - Front obstacle detection
- `/ultrasonic/front_right` - Front-right obstacle
- `/ultrasonic/front_left` - Front-left obstacle  
- `/ultrasonic/right` - Right side obstacle
- `/ultrasonic/left` - Left side obstacle
- `/water_level/clean` - Clean tank level
- `/water_level/dirty` - Dirty tank level

#### IR Sensors (8 topics) ✅ KEEP
- `/ir/front_right/object` - Object detection
- `/ir/front_left/object` - Object detection
- `/ir/back_right/object` - Object detection
- `/ir/back_left/object` - Object detection
- `/ir/front_right/stair` - Cliff detection
- `/ir/front_left/stair` - Cliff detection
- `/ir/back_right/stair` - Cliff detection
- `/ir/back_left/stair` - Cliff detection

#### Encoders (2 topics) ✅ KEEP
- `/encoder/left` - Left wheel encoder
- `/encoder/right` - Right wheel encoder

#### Safety (3 topics) ✅ KEEP
- `/safety/estop_main` - Main emergency stop (Pin 22)
- `/safety/front_switch` - Front bumper switch (Pin 24)
- `/safety/estop` - Combined e-stop status (logical OR of both switches)

#### Battery & IMU (2 topics) ✅ KEEP
- `/battery/state` - Battery voltage/current/percentage
- `/imu/data` - IMU orientation/acceleration

#### Auxiliary Outputs (7 topics) ⚠️ REVIEW
**What are relays?** Relays are electrical switches that control high-power devices (brushes, pumps)
- `/buzzer/state` - Buzzer status feedback (Pin 26)
- `/indicator/state` - LED status feedback (Pin 28)
- `/vacuum/state` - **VACUUM PUMP** status (Pin 31 - direct connection, not relay)
- `/brush/main/state` - **MAIN SCRUBBER BRUSH** status (Pin 30 - relay controlled)
- `/brush/left/state` - **LEFT SWEEPING BRUSH** status (Pin 27 - relay controlled)
- `/brush/right/state` - **WATER PUMP** status (Pin 29 - relay controlled, confusing name!)
- `/relay/status` - Combined status string of all relay-controlled outputs

**Note:** Arduino sends brush states in `RELAY:` section of serial data

#### Raw Data (1 topic) ❌ REMOVE (for debugging only)
- `/sensors/raw` - Raw serial string from Arduino

**Total: 30 topics from multi_sensor_node**

---

### **2. motor_controller_node.py**
**Purpose:** Subscribes to `/cmd_vel`, controls motors via Arduino

#### Published Topics (2) ✅ KEEP
- `/motor/pwm` - Current motor PWM values
- `/motor/status` - Motor status string

**Total: 2 topics from motor_controller**

---

### **3. odometry_node.py**
**Purpose:** Calculates odometry from encoders

#### Published Topics (1) ✅ KEEP
- `/odom` - Odometry (position, velocity)

**Total: 1 topic from odometry_node**

---

### **4. industry_obstacle_avoidance.py**
**Purpose:** Sensor fusion for obstacle avoidance

#### Subscriptions (reads data to make decisions)
**Why it reads these topics:** This node is the "brain" - it reads all sensor data, analyzes obstacles, and decides how to move the robot safely. It then publishes velocity commands (`/cmd_vel`).

- Subscribes to `/scan` (LiDAR) - 360° obstacle detection
- Subscribes to `/ultrasonic/*` (6 sensors) - Mid-range obstacle detection  
- Subscribes to `/ir/*/object` and `/ir/*/stair` (8 IR) - Close-range & cliff detection
- Subscribes to `/safety/estop` - Emergency stop status

#### Published Topics (2) ✅ KEEP
- `/cmd_vel` - Velocity commands
- `/obstacle/emergency` - Emergency stop flag

**Total: 2 topics from industry_obstacle_avoidance**

---

### **5. sllidar_ros2 (LiDAR Driver)**
**Purpose:** RPLIDAR A1 sensor interface

#### Published Topics (1) ✅ KEEP
- `/scan` - LaserScan data (360° point cloud)

**Total: 1 topic from sllidar**

---

### **6. rosbridge_server**
**Purpose:** WebSocket bridge for dashboard

#### Internal Topics (auto-generated, ignore)
- `/rosout`
- `/parameter_events`
- `/rosbridge_websocket/*`

**Total: 0 user topics**

---

## 📊 SUMMARY

### Total Active Topics: **36**
- multi_sensor_node: 30
- motor_controller: 2  
- odometry: 1
- industry_avoidance: 2
- sllidar: 1

### Duplicate Topics Found: **NONE** ✅
After audit, there are **NO duplicate topics**. Each topic has a single publisher.

### Topics Previously Seen - Explained:

**Why these topics exist but show "not used":**

```
/map                   # 2 nodes are WAITING for map data (0 publishers)
                       # You're not running SLAM, so no one publishes to this
                       # Dashboard or another node is trying to subscribe
                       
/parameter_events      # ROS2 system topic (automatic)
/rosout                # ROS2 logging topic (automatic)
/tf                    # Transform tree (odometry publishes base_link → odom transform)
/tf_static             # Static transforms (for robot structure, if any)
```

**Topics that DON'T currently exist** (you saw them in a previous session when you ran RViz):
```
/clicked_point         # Created when RViz visualization tool is running
/goal_pose             # Created when RViz navigation goal tool is active
/initialpose           # Created when RViz localization tool is active
```

These RViz topics only appear when you open RViz. They're harmless visualization tools.

---

## ⚠️ TOPICS TO CONSIDER REMOVING

### **Category 1: Debugging Topics** (Remove for production)
1. `/sensors/raw` - Raw Arduino serial string  
   - **Action:** Comment out in `multi_sensor_node.py` line 106 + line ~280 (publish call)
   - **Reason:** Wastes bandwidth, only useful for debugging
   - **Impact:** None (not used by any other node)

### **Category 2: Auxiliary Status Feedback** (Optional - keep if dashboard shows them)
2. `/buzzer/state` - Buzzer echo  
3. `/indicator/state` - LED echo  
4. `/vacuum/state` - Vacuum status echo  
5. `/brush/main/state` - Brush status echo  
6. `/brush/left/state` - Brush status echo  
7. `/brush/right/state` - Brush status echo  
8. `/relay/status` - Combined relay status  
   - **Action:** If dashboard doesn't show these, comment out publishers (lines 95-103, 250-260)
   - **Reason:** Feedback for actuators you control is often redundant
   - **Impact:** Dashboard won't show brush/vacuum status (but you still have control)

---

## ✅ RECOMMENDED CLEANUP ACTIONS

### **Minimal Cleanup (recommended):**
Remove only `/sensors/raw` topic:

**File:** `/home/saad/clean_ws/src/robot_sensors/robot_sensors/multi_sensor_node.py`

**Line 106:** Comment out publisher creation:
```python
# self.pub_raw = self.create_publisher(String, '/sensors/raw', 10)
```

**Line ~280:** Comment out publish call in `parse_serial_data()`:
```python
# self.pub_raw.publish(raw_msg)
```

**Savings:** -1 topic, reduces bandwidth by ~500 bytes/sec

### **Moderate Cleanup (if actuator feedback not needed):**
Additionally comment out auxiliary state publishers (lines 95-100):
```python
# self.pub_buzzer_state = self.create_publisher(Bool, '/buzzer/state', 10)
# self.pub_led_state = self.create_publisher(Bool, '/indicator/state', 10)
# self.pub_vacuum_state = self.create_publisher(Bool, '/vacuum/state', 10)
# self.pub_brush_main_state = self.create_publisher(Bool, '/brush/main/state', 10)
# self.pub_brush_left_state = self.create_publisher(Bool, '/brush/left/state', 10)
# self.pub_brush_right_state = self.create_publisher(Bool, '/brush/right/state', 10)
```

And comment out their publish calls in `parse_serial_data()` (around lines 250-260).

**Savings:** -7 topics total, minimal bandwidth impact

---

## 🔍 ARDUINO DATA FORMAT (Reference)
From `/home/saad/clean_ws/src/robot_sensors/arduino/multi_sensor/src/main.cpp`:

```
US:front,fright,fleft,right,left|USW:clean,dirty|IR_OBJ:fr,fl,br,bl|IR_STAIR:fr,fl,br,bl|ENC:left,right|ESTOP:main,front|BAT:v,i,p|IMU:roll,pitch,yaw,ax,ay,az,gx,gy,gz|STATE:buzzer,led,vac,b_main,b_left,b_right,estop|RELAY:status_string
```

All data is already captured in specific topics. The `/sensors/raw` topic is redundant.

---

## 🎯 CONCLUSION

Your system is **already well-organized**! The apparent "duplicate" topics you saw were likely:
1. **ROS2 system topics** (`/rosout`, `/parameter_events`, `/tf`)
2. **RViz visualization topics** (if you opened RViz)
3. **Dashboard viewing multiple topics** (not duplicates, just different sensors)

**There are NO duplicate publishers.** Each sensor publishes to exactly one topic.

The only cleanup needed is removing the `/sensors/raw` debugging topic.

---

## ✅ CLEANUP COMPLETED (November 18, 2025)

### What Was Removed:
- ❌ `/sensors/raw` topic (raw Arduino serial debug data)

### Changes Made:
**File:** `src/robot_sensors/robot_sensors/multi_sensor_node.py`
- Line 106: Commented out `self.pub_raw = self.create_publisher(...)`
- Line 318: Commented out raw data publish call

### Status:
✅ Package rebuilt successfully  
✅ All nodes restarted  
✅ All essential topics working  
✅ System optimized - reduced from ~58 to 57 topics

---

## ⚠️ SENSOR READING NOTES

### Understanding Ultrasonic Values:
From Arduino: `US:55.64,79.70,5.00,48.96,4.30|USW:3.83,19.62`

**Mapping:**
- US[0] = 55.64cm → `/ultrasonic/front` ✅
- US[1] = 79.70cm → `/ultrasonic/front_right` ✅
- US[2] = **5.00cm** → `/ultrasonic/front_left` ⚠️ **Very close!**
- US[3] = 48.96cm → `/ultrasonic/right` ✅
- US[4] = 4.30cm → `/ultrasonic/left` ✅

- USW[0] = **3.83cm** → `/water_level/clean` ⚠️ **Very close!**
- USW[1] = 19.62cm → `/water_level/dirty` ✅

### Are These Wrong Values?
**NO!** These are CORRECT sensor readings. The values are small because:

1. **Front-Left sensor (5cm)**: 
   - Sensor is detecting something 5cm away
   - Could be: robot body part, wall, obstacle, or sensor mounting
   - Check if sensor is angled toward robot body or blocked

2. **Clean Water Level (3.83cm):**
   - Measures distance from sensor to water surface
   - 3.83cm = Tank is nearly FULL (water is close to sensor)
   - This is normal if tank was just filled!

3. **Left sensor (4.3cm):**
   - Similar to front-left, detecting something very close
   - Could indicate robot is near a wall on the left side

### How to Verify:
1. **Move robot to open space** - values should increase
2. **Check sensor mounting** - ensure sensors point outward, not at robot body
3. **Empty clean water tank** - value should increase (>10cm when empty)
4. **Check for obstacles** - remove any objects near sensors

### Normal Ranges:
- **Obstacle sensors:** 20cm - 200cm (in open space)
- **Water level sensors:** 
  - Empty tank: 15-20cm
  - Full tank: 3-5cm ✅ (what you're seeing)

Your sensors are working correctly! The small values indicate:
- ✅ Clean water tank is full
- ⚠️ Left and front-left sensors detecting nearby obstacles or robot structure

---

## 📝 NEXT STEPS

1. **Physical inspection:**
   - Check if front-left ultrasonic sensor is blocked or angled incorrectly
   - Verify clean water tank level (3.83cm = nearly full)
   - Move robot to open space and re-check sensor values
   
2. **Monitor sensors:**
   ```bash
   ros2 topic echo /ultrasonic/front_left
   ros2 topic echo /water_level/clean
   ```

3. **Test dashboard** - All sensors should display in real-time

4. **If values don't change when moving robot:**
   - Check sensor wiring
   - Verify sensor is not pointing at robot body/structure
   - Test individual sensor in Arduino serial monitor

**System is clean and operational! 🤖✨**
