#!/bin/bash
# Test script for robot motors through ROS2

echo "=== Robot Motor Test ==="
echo ""

# Source ROS2
source /opt/ros/humble/setup.bash
source /home/saad/clean_ws/install/setup.bash

echo "1. Checking active ROS2 nodes..."
ros2 node list
echo ""

echo "2. Checking available topics..."
ros2 topic list | grep -E "cmd_vel|motor"
echo ""

echo "3. Sending motor command for 3 seconds..."
timeout 3 ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

echo ""
echo "4. Stopping motors..."
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

echo ""
echo "Done! Did the motors move?"
