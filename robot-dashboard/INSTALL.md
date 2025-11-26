# Quick Installation Guide

## Step 1: Install Dependencies

```bash
cd /home/saad/clean_ws/robot-dashboard
npm install
```

This will install all required packages (React, TypeScript, ROS bridge, etc.)

## Step 2: Start ROS Bridge

In a separate terminal:

```bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

If not installed:
```bash
sudo apt install ros-humble-rosbridge-suite
```

## Step 3: Start Dashboard

```bash
npm run dev
```

Dashboard will be available at: **http://192.168.0.181:3000**

## Features Included

✅ **Home Page** - Real-time monitoring, map, LIDAR, status
✅ **Manual Control** - Joystick, speed control, sensor display
✅ **Auto Clean** - Start/stop autonomous cleaning
✅ **Map View** - Interactive map with robot position
✅ **Diagnostics** - Sensor monitoring and health
✅ **Settings** - Robot configuration

## Current Robot Integration

The dashboard connects to your existing robot system:
- Wall-following cleaner: `/wall_follow_cleaner` node
- SLAM mapping: `/slam_toolbox` node  
- Safety monitor: `/safety_monitor` node
- All sensor topics already publishing

## Quick Test

1. Make sure your robot system is running:
   ```bash
   ./start_wall_follow_cleaner.sh
   ```

2. Launch dashboard (separate terminal):
   ```bash
   cd robot-dashboard
   npm run dev
   ```

3. Open browser: http://192.168.0.181:3000

You should see live robot data immediately!
