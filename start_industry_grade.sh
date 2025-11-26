#!/bin/bash

# Industry-Grade Obstacle Avoidance System Startup Script
# =========================================================
# Multi-sensor fusion for professional navigation:
# - LiDAR: 360° long-range detection
# - 6 Ultrasonic: Mid-range precision (left excluded - faulty)
# - 8 IR: Close-range + cliff detection
# - Sensor fusion with confidence scoring

set -e

echo "🏭 INDUSTRY-GRADE OBSTACLE AVOIDANCE SYSTEM"
echo "=========================================="
echo ""

# Navigate to workspace
cd /home/saad/clean_ws

# Source ROS2 environment
echo "📦 Sourcing ROS2 environment..."
source /opt/ros/humble/setup.bash
source install/setup.bash

# Detect LiDAR serial port (handles flaky /dev/ttyUSB ordering)
LIDAR_PORT=${LIDAR_PORT:-/dev/ttyUSB0}
if [ ! -e "$LIDAR_PORT" ]; then
    if [ -e /dev/rplidar ]; then
        LIDAR_PORT=/dev/rplidar
    elif [ -e /dev/ttyUSB1 ]; then
        LIDAR_PORT=/dev/ttyUSB1
    fi
fi

if [ ! -e "$LIDAR_PORT" ]; then
    echo "⚠️  LiDAR serial port not detected (checked /dev/ttyUSB0,/dev/ttyUSB1,/dev/rplidar)"
    echo "    Plug in the LiDAR USB cable, then re-run this script."
    exit 1
fi

echo "📡 Using LiDAR port: $LIDAR_PORT"

# Kill any existing nodes
echo "🧹 Cleaning up old processes..."
pkill -f multi_sensor_node || true
pkill -f motor_controller_node || true
pkill -f odometry_node || true
pkill -f sllidar_node || true
pkill -f industry_obstacle_avoidance || true
pkill -f rosbridge || true
sleep 2

echo ""
echo "🚀 Starting system components..."
echo ""

# 1. Multi-sensor node (Arduino interface)
echo "1️⃣ Starting Multi-Sensor Node (7 US, 8 IR, IMU, Battery)..."
nohup ros2 run robot_sensors multi_sensor_node > /tmp/multi_sensor.log 2>&1 &
MULTI_SENSOR_PID=$!
echo "   ✓ PID: $MULTI_SENSOR_PID"
sleep 2

# 2. LiDAR node
echo "2️⃣ Starting LiDAR Node (RPLIDAR A1)..."
nohup ros2 launch sllidar_ros2 sllidar_a1_launch.py serial_port:=${LIDAR_PORT} > /tmp/lidar.log 2>&1 &
LIDAR_PID=$!
echo "   ✓ PID: $LIDAR_PID"
sleep 3

# 3. Motor controller
echo "3️⃣ Starting Motor Controller..."
nohup ros2 run robot_sensors motor_controller_node > /tmp/motor.log 2>&1 &
MOTOR_PID=$!
echo "   ✓ PID: $MOTOR_PID"
sleep 1

# 4. Odometry node
echo "4️⃣ Starting Odometry Node..."
nohup ros2 run robot_sensors odometry_node > /tmp/odometry.log 2>&1 &
ODOM_PID=$!
echo "   ✓ PID: $ODOM_PID"
sleep 1

# 5. Industry-grade obstacle avoidance (NEW!)
echo "5️⃣ Starting Industry-Grade Obstacle Avoidance..."
nohup ros2 run robot_sensors industry_obstacle_avoidance > /tmp/industry_avoidance.log 2>&1 &
AVOIDANCE_PID=$!
echo "   ✓ PID: $AVOIDANCE_PID"
sleep 1

# 6. Rosbridge for web dashboard
echo "6️⃣ Starting Rosbridge WebSocket..."
nohup ros2 launch rosbridge_server rosbridge_websocket_launch.xml > /tmp/rosbridge.log 2>&1 &
ROSBRIDGE_PID=$!
echo "   ✓ PID: $ROSBRIDGE_PID"
sleep 1

echo ""
echo "✅ All systems running!"
echo ""
echo "📊 SYSTEM STATUS:"
echo "   Multi-Sensor:  PID $MULTI_SENSOR_PID (log: /tmp/multi_sensor.log)"
echo "   LiDAR:         PID $LIDAR_PID (log: /tmp/lidar.log)"
echo "   Motor:         PID $MOTOR_PID (log: /tmp/motor.log)"
echo "   Odometry:      PID $ODOM_PID (log: /tmp/odometry.log)"
echo "   Avoidance:     PID $AVOIDANCE_PID (log: /tmp/industry_avoidance.log)"
echo "   Rosbridge:     PID $ROSBRIDGE_PID (log: /tmp/rosbridge.log)"
echo ""
echo "🎯 ACTIVE SENSORS:"
echo "   ✓ LiDAR (360° coverage, 0.1-12m)"
echo "   ✓ 6 Ultrasonic (Front, F-Right, F-Left, Right) [Left excluded]"
echo "   ✓ 8 IR (4 object + 4 cliff detection)"
echo "   ✓ IMU (orientation)"
echo "   ✓ Battery monitoring (voltage, current)"
echo ""
echo "🎛️ CONTROLS:"
echo "   Dashboard: http://localhost:3000"
echo "   Foxglove:  ws://localhost:9090"
echo ""
echo "📝 MONITORING:"
echo "   - Watch logs: tail -f /tmp/industry_avoidance.log"
echo "   - Check topics: ros2 topic list"
echo "   - View sensor data: ros2 topic echo /scan"
echo ""
echo "🛑 STOP SYSTEM:"
echo "   pkill -f 'multi_sensor_node|motor_controller_node|industry_obstacle_avoidance|sllidar_node'"
echo ""
echo "=========================================="
echo "🏭 Industry-Grade System Ready!"
echo "=========================================="
