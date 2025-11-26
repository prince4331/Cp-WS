# 🚀 Quick Start Guide

**For full documentation, see:** `ROBOT_SYSTEM_GUIDE.md`

---

## ⚡ Start Robot (1 command)

```bash
cd /home/saad/clean_ws && ./start_industry_grade.sh
```

## 🎮 Open Dashboard

```bash
# In new terminal
cd /home/saad/clean_ws/robot-dashboard
npm run dev -- --host 192.168.0.181 --port 3000
```

Then open: **http://192.168.0.181:3000**

## 🛑 Stop Robot

```bash
pkill -f "ros2|python3.*robot_sensors"
```

---

## 📋 Common Tasks

### Manual Control
```bash
./start_industry_grade.sh
pkill -f industry_obstacle_avoidance  # Disable obstacle avoidance
# Open dashboard at http://192.168.0.181:3000
```

### Create Map
```bash
./create_map_with_keyboard.sh
# Use WASD to drive, saves map on exit
```

### Check Status
```bash
ros2 node list              # List all nodes
ros2 topic list             # List all topics
ros2 topic hz /scan         # Check LIDAR rate
netstat -tuln | grep 9090   # Check rosbridge
```

---

## 🆘 Quick Troubleshooting

**Dashboard shows no data?**
```bash
pkill -f multi_sensor
ros2 run robot_sensors multi_sensor_node
```

**Robot not moving?**
```bash
pkill -f obstacle_avoidance  # Stop obstacle avoidance
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.2}}" --once
```

**LIDAR not working?**
```bash
sudo chmod 666 /dev/ttyUSB0
pkill -f sllidar
ros2 launch sllidar_ros2 sllidar_a1_launch.py
```

---

## 📖 More Help

- Full guide: `cat ROBOT_SYSTEM_GUIDE.md`
- Cleanup details: `cat WORKSPACE_CLEANUP_SUMMARY.md`
- Logs: `/tmp/*.log`

---

**Last Updated:** November 2025
