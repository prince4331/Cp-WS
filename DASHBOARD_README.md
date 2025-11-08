# 🖥️ Real-Time Sensor Dashboard

A beautiful terminal-based dashboard for monitoring all robot sensors in real-time.

## Features

✨ **Live sensor monitoring** with color-coded status indicators
- 📏 5x Ultrasonic sensors (distance in cm)
- 🔴 4x IR Object Detection sensors 
- ⚠️  4x IR Stair Detection sensors
- ⚙️  2x Wheel Encoders (tick counts)
- 🧭 Odometry (position, velocity)
- 📡 LIDAR scan statistics

🎨 **Color-coded display:**
- 🟢 Green = Safe/Normal
- 🔴 Red = Detected/Warning
- 🔵 Cyan = Odometry data
- 🟣 Magenta = LIDAR data
- 🟡 Yellow = Section headers

⚡ **Real-time updates** at ~10 Hz refresh rate

## Quick Start

### Method 1: Quick Launch Script
```bash
cd /home/saad/clean_ws
./dashboard.sh
```

### Method 2: Direct ROS Command
```bash
cd /home/saad/clean_ws
source install/setup.bash
ros2 run robot_sensors sensor_dashboard
```

### Method 3: Web Dashboard (browser-based)
```bash
cd /home/saad/clean_ws
source install/setup.bash
ros2 run robot_sensors web_dashboard --port 8080
```

Then open `http://<robot-ip>:8080/` in your browser to see live sensor values and toggle actuators.

## Prerequisites

**The robot system must be running first!**

In a separate terminal:
```bash
cd /home/saad/clean_ws
source install/setup.bash
ros2 launch robot_sensors complete_robot_launch.py
```

## Dashboard Layout

```
╔═══════════════════════════════════════════════════════════════════╗
║           🤖 FLOOR CLEANING ROBOT - SENSOR DASHBOARD 🤖          ║
║                    Last Update: 15:45:23.456                      ║
╠═══════════════════════════════════════════════════════════════════╣
║                                                                   ║
║  📏 ULTRASONIC SENSORS              ⚙️  WHEEL ENCODERS           ║
║    front       : 125.3 cm             Left:     12345            ║
║    front_left  :  45.2 cm             Right:    12389            ║
║    front_right :  48.1 cm                                        ║
║    left        :  30.5 cm           🧭 ODOMETRY                  ║
║    right       :  32.8 cm             X:      0.534 m            ║
║                                       Y:      0.123 m            ║
║  🔴 IR OBJECT DETECTION               Theta:   45.2°             ║
║    front       : Clear                Lin V:  0.150 m/s          ║
║    front_left  : Clear                Ang V:  0.000 rad/s        ║
║    front_right : DETECTED                                        ║
║    back        : Clear              📡 LIDAR SCAN                ║
║                                       Points:     360            ║
║  ⚠️  IR STAIR DETECTION               Min Range:  0.15 m         ║
║    front       : Safe                 Max Range: 11.85 m         ║
║    front_left  : Safe                 Scan Time:  5.2 ms         ║
║    front_right : Safe                                            ║
║    back        : Safe                                            ║
║                                                                   ║
╠═══════════════════════════════════════════════════════════════════╣
║  📊 RAW SENSOR DATA:                                              ║
║  US:125.3,45.2,48.1,30.5,32.8|IR_OBJ:0,0,1,0|IR_STAIR:0,0,0,0|...║
╠═══════════════════════════════════════════════════════════════════╣
║  Press 'q' to quit | Press 'r' to reset encoders                 ║
╚═══════════════════════════════════════════════════════════════════╝
```

## Controls

- **'q' or 'Q'**: Quit the dashboard
- **Ctrl+C**: Emergency stop

## Monitoring Tips

### 🟢 Normal Operation
- Ultrasonic sensors showing distances > 10 cm
- IR object sensors showing "Clear"
- IR stair sensors showing "Safe"
- Encoders incrementing smoothly
- LIDAR scanning with 360 points

### 🔴 Alert Conditions
- Ultrasonic < 10 cm = Obstacle very close
- IR Object "DETECTED" = Object in path
- IR Stair "STAIR!" = Edge/stair detected (STOP!)
- Encoders not changing = Wheels stuck
- LIDAR points < 100 = Scan issue

### 🧭 Movement Verification
Watch odometry values while sending movement commands:
```bash
# In another terminal:
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}, angular: {z: 0.0}}"
```

You should see:
- Lin V changing to match commanded velocity
- X/Y position changing
- Encoder values incrementing

## Troubleshooting

### Dashboard shows no data (all "---")
- Check if robot system is running: `ros2 topic list`
- Verify topics are publishing: `ros2 topic hz /ultrasonic/front`
- Check Arduino connection: `ls -l /dev/ttyACM*`

### Dashboard looks broken/garbled
- Your terminal may be too small. Resize to at least 100x30
- Try different terminal emulator (gnome-terminal, konsole, xterm)

### Updates are slow/frozen
- Check CPU usage: `top`
- Verify ROS nodes are running: `ros2 node list`
- Check for serial port errors in main system terminal

### Colors not showing
- Your terminal may not support colors
- Try: `export TERM=xterm-256color`

## Technical Details

**Update Rate**: ~10 Hz (100ms refresh)
**ROS Topics Monitored**: 24 topics
- `/ultrasonic/*` (5 topics)
- `/ir_object/*` (4 topics)
- `/ir_stair/*` (4 topics)
- `/encoder/*` (2 topics)
- `/odom` (1 topic)
- `/scan` (1 topic)
- `/sensors/raw` (1 topic)
- System topics (6 topics)

**Dependencies**:
- Python 3 curses library (standard library)
- ROS 2 rclpy
- sensor_msgs, nav_msgs, std_msgs

## Integration with Other Tools

### Record Dashboard Session
```bash
# Install asciinema for terminal recording
sudo apt install asciinema

# Record dashboard
asciinema rec dashboard_session.cast
./dashboard.sh
# Press Ctrl+D to stop recording
```

### Monitor Multiple Screens
Use tmux/screen to see dashboard + logs:
```bash
tmux new-session \; \
  split-window -h \; \
  send-keys 'cd /home/saad/clean_ws && source install/setup.bash && ros2 launch robot_sensors complete_robot_launch.py' C-m \; \
  select-pane -t 0 \; \
  send-keys 'cd /home/saad/clean_ws && ./dashboard.sh' C-m
```

## Next Steps

After verifying sensors:
1. Test movement commands while watching odometry
2. Create obstacle avoidance logic
3. Implement stair detection safety stops
4. Add autonomous navigation

## Support

For issues or questions:
- Check main README: `/home/saad/clean_ws/README.md`
- System status: `./show_status.sh`
- Topic verification: `ros2 topic list`

---

**Happy monitoring! 🤖📊**
