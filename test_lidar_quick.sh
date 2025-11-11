#!/bin/bash

# Quick test of LIDAR with external motor control
# Run this when motor is already spinning

cd /home/saad/clean_ws
source install/setup.bash

echo "=========================================="
echo "Testing LIDAR with External Motor"
echo "=========================================="
echo ""
echo "IMPORTANT: Ensure LIDAR motor is spinning NOW!"
echo ""
read -p "Press ENTER when motor is spinning... " 

echo ""
echo "Starting LIDAR driver (without motor control)..."
echo ""

ros2 launch sllidar_ros2 sllidar_a2m7_external_motor_launch.py \
    serial_port:=/dev/ttyUSB0 \
    serial_baudrate:=115200 \
    auto_motor_ctrl:=false
