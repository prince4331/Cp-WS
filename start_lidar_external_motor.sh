#!/bin/bash

# Script to start LIDAR with external motor control
# This keeps motor power ON while launching the ROS driver

echo "================================================"
echo "Starting LIDAR with External Motor Control"
echo "================================================"

cd /home/saad/clean_ws

# Source ROS2 environment
source install/setup.bash

# Ensure motor is powered (you may need to adjust this based on your setup)
echo "Ensure LIDAR motor is spinning BEFORE continuing..."
echo "Press ENTER when motor is spinning steadily"
read

# Launch LIDAR driver WITHOUT motor control
echo ""
echo "Launching LIDAR driver (no motor control)..."
ros2 launch sllidar_ros2 sllidar_a2m7_external_motor_launch.py \
    serial_port:=/dev/ttyUSB0 \
    serial_baudrate:=115200 \
    scan_mode:=Standard

echo ""
echo "LIDAR driver stopped"
