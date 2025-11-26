# Industrial-Grade Robot Dashboard - Complete Setup Guide

## 🚀 Quick Start

```bash
cd /home/saad/clean_ws/robot-dashboard
npm install
npm run dev
```

Dashboard will be available at: **http://192.168.0.181:3000**

## 📁 Project Structure

```
robot-dashboard/
├── package.json
├── tsconfig.json
├── tailwind.config.js
├── vite.config.ts
├── index.html
├── src/
│   ├── main.tsx
│   ├── App.tsx
│   ├── index.css
│   ├── types/
│   │   ├── ros.ts              # ROS message types
│   │   └── robot.ts            # Robot state types
│   ├── hooks/
│   │   ├── useROS.ts           # ROS connection hook
│   │   ├── useRobotState.ts    # Robot state management
│   │   ├── useSensors.ts       # Sensor data hook
│   │   └── useMap.ts           # Map data hook
│   ├── store/
│   │   └── robotStore.ts       # Zustand state management
│   ├── lib/
│   │   ├── rosbridge.ts        # ROS bridge connection
│   │   ├── mapRenderer.ts      # Canvas map rendering
│   │   └── utils.ts            # Utility functions
│   ├── components/
│   │   ├── layout/
│   │   │   ├── Sidebar.tsx
│   │   │   ├── Header.tsx
│   │   │   └── Layout.tsx
│   │   ├── widgets/
│   │   │   ├── BatteryIndicator.tsx
│   │   │   ├── StatusCard.tsx
│   │   │   ├── SensorBar.tsx
│   │   │   ├── MapCanvas.tsx
│   │   │   ├── LidarVisualizer.tsx
│   │   │   ├── Joystick.tsx
│   │   │   ├── RobotIcon.tsx
│   │   │   └── EmergencyStop.tsx
│   │   └── common/
│   │       ├── Button.tsx
│   │       ├── Card.tsx
│   │       ├── Toggle.tsx
│   │       ├── Slider.tsx
│   │       └── Modal.tsx
│   └── pages/
│       ├── HomePage.tsx         # 1. Monitoring dashboard
│       ├── ManualControl.tsx    # 2. Manual control
│       ├── AutoClean.tsx        # 3. Automatic cleaning
│       ├── MapPage.tsx          # 4. Map & navigation
│       ├── Scheduling.tsx       # 5. Cleaning schedules
│       ├── Diagnostics.tsx      # 6. Sensor diagnostics
│       ├── Settings.tsx         # 7. System settings
│       └── Logs.tsx             # 8. Mission logs
```

## 🔌 ROS2 Setup Required

### 1. Install rosbridge_suite

```bash
sudo apt install ros-humble-rosbridge-suite
```

### 2. Launch rosbridge server

```bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

Default port: **9090**

### 3. Required ROS2 Topics

The dashboard expects these topics to be published:

**Sensor Topics:**
- `/scan` (sensor_msgs/LaserScan) - LIDAR data
- `/ultrasonic_sensors` (std_msgs/Int32MultiArray) - [front, left_corner, right_corner, left_side, right_side]
- `/ir_sensors` (std_msgs/Int32MultiArray) - [4 obstacle sensors + 4 cliff sensors]
- `/imu` (sensor_msgs/Imu) - IMU data
- `/odom` (nav_msgs/Odometry) - Robot odometry
- `/battery` (sensor_msgs/BatteryState) - Battery status
- `/emergency_stop` (std_msgs/Bool) - Emergency stop state
- `/bumper` (std_msgs/Bool) - Bumper switch

**State Topics:**
- `/robot_state` (std_msgs/String) - Current mode: IDLE/MANUAL/AUTO/ERROR
- `/mission_status` (std_msgs/String) - Mission status
- `/cleaning_status` (std_msgs/String) - Cleaning state
- `/coverage_path` (nav_msgs/Path) - Planned coverage path

**Map Topics:**
- `/map` (nav_msgs/OccupancyGrid) - SLAM map

**Command Topics (Dashboard publishes to):**
- `/cmd_vel` (geometry_msgs/Twist) - Velocity commands
- `/cleaning_command` (std_msgs/String) - start/stop/pause commands
- `/actuator_command` (std_msgs/String) - brush/suction control

## 🎨 Features Overview

### 1. HOME SCREEN
- Live map with robot position
- Real-time LIDAR visualization
- Battery status & mode indicators
- Emergency stop button
- Start/pause/stop controls
- Mission progress metrics

### 2. MANUAL CONTROL
- Virtual joystick (touch & mouse)
- Direction buttons
- Speed control slider
- Live sensor distances displayed around robot icon
- Actuator controls (brush, suction)

### 3. AUTOMATIC CLEANING
- Auto clean, spot clean, edge clean modes
- Return to home button
- Live coverage statistics
- Area cleaned (m²)
- Path visualization overlay

### 4. MAP & NAVIGATION
- Interactive map canvas
- Set initial pose
- Define restricted zones
- Virtual walls
- Zoom/pan controls
- Save map snapshots

### 5. SCHEDULING
- Daily/weekly cleaning schedules
- Time picker for each schedule
- Mode selection
- Enable/disable schedules
- Persistent storage

### 6. SENSOR DIAGNOSTICS
- Real-time sensor graphs (Recharts)
- Green/yellow/red status indicators
- Raw sensor value displays
- Update frequency (Hz)
- Sensor health monitoring

### 7. SYSTEM SETTINGS
- Robot parameters (speed, width, spacing)
- Network settings
- Firmware update upload
- Factory reset
- Sound settings

### 8. LOGS PAGE
- Mission history table
- Error logs with timestamps
- Export to CSV
- Searchable and filterable

## 🎯 Key Technologies Used

- **React 18** - UI framework
- **TypeScript** - Type safety
- **Vite** - Build tool
- **TailwindCSS** - Styling
- **Framer Motion** - Animations
- **Recharts** - Data visualization
- **roslib.js** - ROS bridge client
- **Zustand** - State management
- **React Router** - Navigation
- **Lucide React** - Icons

## 🔧 Configuration

### WebSocket Connection

Edit `src/lib/rosbridge.ts`:

```typescript
const ROSBRIDGE_URL = 'ws://192.168.0.181:9090';
```

### Dashboard Port

Edit `vite.config.ts`:

```typescript
server: {
  host: '0.0.0.0',
  port: 3000,
}
```

## 📱 Responsive Design

Optimized for:
- Desktop: 1920x1080
- Tablet: 1024x600 (primary target)
- Mobile: 768x1024

## 🚨 Emergency Stop Integration

The emergency stop button publishes to `/emergency_stop` topic:
- Pressed: `std_msgs/Bool { data: true }`
- Released: `std_msgs/Bool { data: false }`

Your robot safety monitor should subscribe to this topic.

## 🗺️ Map Rendering

The map canvas uses HTML5 Canvas API to render:
1. Occupancy grid from `/map` topic
2. Robot pose from `/odom` topic
3. LIDAR scan points from `/scan` topic
4. Planned paths from `/coverage_path` topic

Rendering is optimized for 10-20 Hz updates.

## 📊 Data Flow

```
ROS2 Robot → rosbridge_server (port 9090) → WebSocket → Dashboard
                                                             ↓
                                                    Zustand Store
                                                             ↓
                                            React Components (UI)
```

## 🛠️ Development

### Run development server
```bash
npm run dev
```

### Build for production
```bash
npm run build
```

### Preview production build
```bash
npm run preview
```

## 🚀 Deployment on Raspberry Pi

### 1. Build the application
```bash
npm run build
```

### 2. Serve with nginx or Python

**Option A: Python HTTP server**
```bash
cd dist
python3 -m http.server 3000 --bind 0.0.0.0
```

**Option B: Nginx**
```bash
sudo apt install nginx
sudo cp -r dist/* /var/www/html/
sudo systemctl restart nginx
```

### 3. Auto-start with systemd

Create `/etc/systemd/system/robot-dashboard.service`:

```ini
[Unit]
Description=Robot Dashboard
After=network.target

[Service]
Type=simple
User=saad
WorkingDirectory=/home/saad/clean_ws/robot-dashboard/dist
ExecStart=/usr/bin/python3 -m http.server 3000 --bind 0.0.0.0
Restart=always

[Install]
WantedBy=multi-user.target
```

Enable and start:
```bash
sudo systemctl enable robot-dashboard
sudo systemctl start robot-dashboard
```

## 🎨 UI Customization

### Change Theme Colors

Edit `tailwind.config.js`:

```javascript
colors: {
  primary: {
    500: '#0ea5e9',  // Main brand color
    600: '#0284c7',  // Hover state
    // ... more shades
  }
}
```

### Modify Dashboard Layout

Edit `src/components/layout/Layout.tsx` for sidebar width, header height, etc.

## 📡 Testing Without Robot

For development without ROS2 running, the dashboard will:
1. Show "Disconnected" status
2. Display mock data in some components
3. Allow UI testing

To enable mock mode, set in `.env`:
```
VITE_MOCK_MODE=true
```

## 🔒 Security Notes

- Dashboard runs on local network only
- No authentication implemented (add if needed)
- WebSocket connection to rosbridge is unencrypted
- Consider using wss:// (WebSocket Secure) for production

## 📚 Additional Resources

- ROS bridge protocol: http://wiki.ros.org/rosbridge_protocol
- roslib.js documentation: http://robotwebtools.org/jsdoc/roslibjs/
- Tailwind CSS: https://tailwindcss.com/docs
- Recharts: https://recharts.org/

## 🐛 Troubleshooting

### Dashboard won't connect to ROS
1. Check rosbridge is running: `ros2 node list | grep rosbridge`
2. Test WebSocket: `wscat -c ws://192.168.0.181:9090`
3. Check firewall: `sudo ufw status`

### Map not displaying
1. Verify `/map` topic: `ros2 topic echo /map --once`
2. Check topic type: `ros2 topic info /map`
3. Ensure SLAM is running

### Sensors not updating
1. Check topic publishing rate: `ros2 topic hz /scan`
2. Verify message format matches dashboard expectations
3. Check browser console for errors

## 📞 Support

For issues specific to your robot:
- Check `/tmp/wall_follow_cleaner.log`
- Verify all ROS nodes are running
- Test topics individually with `ros2 topic echo`

---

**Version:** 1.0.0  
**Created:** November 15, 2025  
**License:** MIT  
**Status:** ✅ Production Ready
