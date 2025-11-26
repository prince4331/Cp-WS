# 🤖 Autonomous Control Guide

## Overview
The autonomous obstacle avoidance system can now be controlled from the dashboard. It starts **DISABLED** by default and only activates when you press the "Start Cleaning" button.

## How It Works

### System Startup
When you run `./start_industry_grade.sh`:
- ✅ All sensors start (LiDAR, Ultrasonic, IR, IMU)
- ✅ Motor controller ready
- ✅ Obstacle avoidance node running
- ⏸️  **Autonomous navigation DISABLED**
- 🎮 Robot responds only to manual dashboard control

### Dashboard Control

#### Auto Clean Page
Navigate to the **Auto Clean** page and use the Mission Control buttons:

1. **Start Cleaning** (Blue button)
   - Enables autonomous navigation
   - Robot starts moving forward with obstacle avoidance
   - Sensors actively prevent collisions
   - Wall-following and coverage cleaning

2. **Pause** (Gray button)
   - Temporarily pauses autonomous movement
   - Robot stops but maintains state
   - Press Start to resume

3. **Stop** (Red button)
   - Disables autonomous navigation
   - Robot stops completely
   - Returns to manual control mode

#### Home Page
The Home page also has Mission Control buttons:
- **Start**: Enable autonomous mode
- **Pause**: Temporarily pause
- **Stop**: Disable autonomous mode

### Manual Control Mode
When autonomous is disabled:
- Use the **Manual Control** page
- Joystick control active
- WASD keyboard controls work
- Robot only moves when you command it

## Technical Details

### Topics
- `/autonomous/enable` (std_msgs/Bool)
  - `true` = Enable autonomous navigation
  - `false` = Disable autonomous navigation

### Command Line Control
You can also control autonomously via command line:

```bash
# Enable autonomous navigation
ros2 topic pub --once /autonomous/enable std_msgs/msg/Bool "{data: true}"

# Disable autonomous navigation
ros2 topic pub --once /autonomous/enable std_msgs/msg/Bool "{data: false}"
```

### Log Monitoring
Watch the autonomous system status:
```bash
tail -f /tmp/industry_avoidance.log
```

You'll see messages like:
- `⏸️  Autonomous control DISABLED - Waiting for dashboard command`
- `🤖 Autonomous navigation ENABLED`
- `⏸️  Autonomous navigation DISABLED`

## Safety Features

Even in autonomous mode:
- ✅ Emergency stop always works
- ✅ Cliff detection (immediate halt)
- ✅ Multi-sensor obstacle avoidance
- ✅ Dynamic speed adjustment based on obstacles
- ✅ Collision prevention at all times

## Usage Scenarios

### Scenario 1: Manual Cleaning
1. Start system: `./start_industry_grade.sh`
2. Open dashboard: http://localhost:3000
3. Go to Manual Control page
4. Use joystick or keyboard to drive
5. Robot moves only when you control it

### Scenario 2: Autonomous Cleaning
1. Start system: `./start_industry_grade.sh`
2. Open dashboard: http://localhost:3000
3. Go to Auto Clean or Home page
4. Press "Start Cleaning" button
5. Robot navigates autonomously with obstacle avoidance

### Scenario 3: Mixed Mode
1. Start in autonomous mode
2. Press "Stop" to disable
3. Switch to Manual Control for precise positioning
4. Return to Auto Clean and press "Start" to resume

## Troubleshooting

### Robot moves on startup
- This should no longer happen! The system starts disabled.
- If it does, press the red "Stop" button immediately

### Dashboard not connecting
- Check ROS bridge is running: `ps aux | grep rosbridge`
- Verify topics: `ros2 topic list`
- Dashboard should show connection status in Settings

### Autonomous not starting
- Check log: `tail /tmp/industry_avoidance.log`
- Verify node is running: `ros2 node list | grep industry`
- Try manual topic command to test

## Dashboard Access

- **Local**: http://localhost:3000
- **Network**: http://192.168.0.166:3000

Enjoy your autonomous cleaning robot! 🚀
