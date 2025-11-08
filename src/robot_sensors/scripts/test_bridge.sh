#!/bin/bash
# Test script for ultrasonic sensor ROS bridge
# Part of autonomous floor cleaning robot project

echo "Testing ultrasonic sensor ROS bridge..."
echo "Make sure Arduino is connected to /dev/ttyACM0"
echo ""

# Source ROS 2 workspace
source /home/saad/clean_ws/install/setup.bash

# Run the serial range node
ros2 run robot_sensors serial_range_node
