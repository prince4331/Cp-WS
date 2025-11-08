#!/bin/bash

# Sensor Dashboard Quick Launch Script
# Displays real-time sensor data in a clean terminal interface

echo "🤖 Starting Sensor Dashboard..."
echo "Make sure the robot system is running first!"
echo ""

cd /home/saad/clean_ws
source install/setup.bash

ros2 run robot_sensors sensor_dashboard
