# 🖥️ Sensor Dashboard - Visual Preview

## What the Dashboard Looks Like

When you run `./dashboard.sh`, you'll see a full-screen terminal interface like this:

```
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
           🤖 FLOOR CLEANING ROBOT - SENSOR DASHBOARD 🤖
                    Last Update: 15:45:23.456
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

📏 ULTRASONIC SENSORS (Range in cm)          ⚙️  WHEEL ENCODERS (Ticks)
  front       :  125.3 cm  [GREEN]             Left:     12345
  front_left  :   45.2 cm  [GREEN]             Right:    12389
  front_right :   48.1 cm  [GREEN]
  left        :   30.5 cm  [GREEN]           🧭 ODOMETRY
  right       :   32.8 cm  [GREEN]             X:      0.534 m
                                               Y:      0.123 m
🔴 IR OBJECT DETECTION                         Theta:   45.2°
  front       :   Clear    [GREEN]             Lin V:  0.150 m/s
  front_left  :   Clear    [GREEN]             Ang V:  0.000 rad/s
  front_right : DETECTED   [RED]
  back        :   Clear    [GREEN]           📡 LIDAR SCAN
                                               Points:     360
⚠️  IR STAIR DETECTION                         Min Range:  0.15 m
  front       :   Safe     [GREEN]             Max Range: 11.85 m
  front_left  :   Safe     [GREEN]             Scan Time:  5.2 ms
  front_right :   Safe     [GREEN]
  back        :   Safe     [GREEN]

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
📊 RAW SENSOR DATA:
US:125.3,45.2,48.1,30.5,32.8|IR_OBJ:0,0,1,0|IR_STAIR:0,0,0,0|ENC:12345,12389
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Press 'q' to quit | Press 'r' to reset encoders
```

## Color Legend

The dashboard uses colors to make monitoring easier:

- 🟢 **GREEN** - Normal/Safe/Clear
  - Ultrasonic sensors with sufficient clearance (> 10 cm)
  - IR sensors showing "Clear" or "Safe"
  
- 🔴 **RED** - Warning/Detected
  - Ultrasonic sensors too close (< 10 cm) 
  - IR showing "DETECTED" or "STAIR!"
  
- 🔵 **CYAN** - Odometry Information
  - Position (X, Y, Theta)
  - Velocities (Linear, Angular)
  - Encoder tick counts
  
- 🟣 **MAGENTA** - LIDAR Information
  - Scan statistics
  - Range data
  
- 🟡 **YELLOW** - Section Headers
  - Sensor category titles

## Live Data Examples

### Example 1: Normal Operation
```
📏 ULTRASONIC SENSORS
  front       :  150.0 cm  ✅ Safe distance
  front_left  :  145.0 cm  ✅ Safe distance
  front_right :  148.0 cm  ✅ Safe distance
  left        :   80.0 cm  ✅ Safe distance
  right       :   85.0 cm  ✅ Safe distance

🔴 IR OBJECT DETECTION
  front       :   Clear    ✅ No obstacles
  front_left  :   Clear    ✅ No obstacles
  front_right :   Clear    ✅ No obstacles
  back        :   Clear    ✅ No obstacles

⚠️  IR STAIR DETECTION
  front       :   Safe     ✅ No stairs/edges
  front_left  :   Safe     ✅ No stairs/edges
  front_right :   Safe     ✅ No stairs/edges
  back        :   Safe     ✅ No stairs/edges
```

### Example 2: Obstacle Detected
```
📏 ULTRASONIC SENSORS
  front       :    8.5 cm  ⚠️  VERY CLOSE!
  front_left  :   45.0 cm  ✅ Safe
  front_right :   48.0 cm  ✅ Safe
  left        :   80.0 cm  ✅ Safe
  right       :   85.0 cm  ✅ Safe

🔴 IR OBJECT DETECTION
  front       : DETECTED   ⛔ Obstacle ahead!
  front_left  :   Clear    ✅ No obstacles
  front_right :   Clear    ✅ No obstacles
  back        :   Clear    ✅ No obstacles
```

### Example 3: Stair Edge Detected (DANGER!)
```
⚠️  IR STAIR DETECTION
  front       :  STAIR!    ⛔ STOP IMMEDIATELY!
  front_left  :  STAIR!    ⛔ Edge detected!
  front_right :   Safe     ✅ No edge
  back        :   Safe     ✅ No edge
```

### Example 4: Robot Moving
```
🧭 ODOMETRY
  X:      1.234 m  ← Moving forward
  Y:      0.567 m  ← Drifting slightly
  Theta:   15.2°   ← Turning
  Lin V:  0.200 m/s ← 20 cm/s forward
  Ang V:  0.150 rad/s ← Rotating

⚙️  WHEEL ENCODERS
  Left:    15234   ← Incrementing
  Right:   15298   ← Incrementing (slightly faster = turning)
```

## How to Use

### Step 1: Start the Robot System
```bash
# Terminal 1
cd /home/saad/clean_ws
source install/setup.bash
ros2 launch robot_sensors complete_robot_launch.py
```

Wait for all nodes to start (you'll see log messages).

### Step 2: Launch the Dashboard
```bash
# Terminal 2 (new terminal)
cd /home/saad/clean_ws
./dashboard.sh
```

The dashboard will appear full-screen.

### Step 3: Monitor Sensors

Watch the values update in real-time (~10 times per second):

- **Ultrasonic values** should change as you move objects near sensors
- **IR sensors** should trigger when objects get close
- **Encoders** should increment when wheels turn
- **Odometry** should update when robot moves
- **LIDAR** should always show ~360 points

### Step 4: Test Movement

While dashboard is running, in a **third terminal**:
```bash
cd /home/saad/clean_ws
source install/setup.bash

# Move forward
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}" --once

# Watch dashboard - you should see:
# - Lin V change to 0.100 m/s
# - X position increasing
# - Encoder values incrementing
```

### Step 5: Exit Dashboard

Press **'q'** to quit cleanly.

## Troubleshooting

### Dashboard shows all "---" (no data)
**Problem**: No sensor data received  
**Solution**: Make sure robot system is running in Terminal 1

### Dashboard looks garbled
**Problem**: Terminal too small  
**Solution**: Resize terminal to at least 100 columns x 30 rows

### Values not updating
**Problem**: Sensors not connected or not publishing  
**Solution**: 
1. Check `ros2 topic hz /ultrasonic/front` - should show ~10 Hz
2. Check Arduino connection: `ls -l /dev/ttyACM0`
3. Check for errors in Terminal 1 (main system)

### Colors not showing
**Problem**: Terminal doesn't support colors  
**Solution**: Try: `export TERM=xterm-256color`

## What to Look For

### ✅ Healthy System Indicators:
- All sensor values updating (not stuck)
- Ultrasonic showing reasonable distances
- Encoders incrementing when wheels move
- LIDAR showing ~360 points
- No RED warnings when area is clear

### ⚠️  Warning Signs:
- Any value stuck at same number = sensor failure
- Ultrasonic always showing max range = no echo received
- Encoders not changing when wheels turn = encoder issue
- LIDAR points < 100 = LIDAR connection problem

## Advanced Usage

### Record a Session
```bash
# Start recording
script -c "./dashboard.sh" dashboard_log.txt

# Later, replay:
cat dashboard_log.txt
```

### Split Screen Monitoring
Use tmux to see dashboard + logs side-by-side:
```bash
tmux new-session \; \
  split-window -h \; \
  send-keys 'ros2 launch robot_sensors complete_robot_launch.py' C-m \; \
  select-pane -t 0 \; \
  send-keys './dashboard.sh' C-m
```

---

## Ready to Launch! 🚀

Now you have a professional-grade monitoring tool for your robot. Start the system and enjoy watching your sensors in action!

```bash
cd /home/saad/clean_ws
./dashboard.sh
```
