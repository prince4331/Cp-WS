#!/bin/bash

# Complete Robot System Launcher
# Starts all ROS2 nodes and the dashboard

echo "=========================================="
echo "  COMPLETE ROBOT SYSTEM LAUNCHER"
echo "=========================================="
echo ""

# Source ROS2 environment
echo "[1/4] Sourcing ROS2 and workspace..."
source /opt/ros/humble/setup.bash
cd /home/saad/clean_ws
source install/setup.bash

# Start ROS2 nodes in background
echo "[2/4] Starting ROS2 nodes..."
echo "  - multi_sensor_node (Arduino bridge)"
echo "  - motor_controller_node (Motor control)"
echo "  - sllidar_ros2 (LiDAR)"

# Start multi_sensor_node
ros2 run robot_sensors multi_sensor_node &
MULTI_PID=$!
sleep 2

# Start motor_controller_node  
ros2 run robot_sensors motor_controller_node &
MOTOR_PID=$!
sleep 1

# Start LiDAR
ros2 launch sllidar_ros2 sllidar_a1_launch.py &
LIDAR_PID=$!
sleep 2

echo ""
echo "ROS2 Nodes Started:"
echo "  - multi_sensor_node (PID: $MULTI_PID)"
echo "  - motor_controller_node (PID: $MOTOR_PID)"
echo "  - sllidar_ros2 (PID: $LIDAR_PID)"
echo ""

# Start the dashboard
echo "[3/4] Starting Robot Dashboard..."
echo ""
echo "=========================================="
echo "  All systems running!"
echo "=========================================="
echo ""
echo "  Dashboard: http://localhost:5000"
echo "  or: http://$(hostname -I | awk '{print $1}'):5000"
echo ""
echo "  Press Ctrl+C to stop all systems"
echo ""

python3 /home/saad/clean_ws/robot_dashboard.py

# Cleanup on exit
echo ""
echo "Shutting down..."
kill $MULTI_PID $MOTOR_PID 2>/dev/null
echo "All systems stopped."
