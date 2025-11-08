#!/bin/bash
# Quick test script for all robot sensors
# Autonomous Floor Cleaning Robot

echo "================================================"
echo "  Autonomous Floor Cleaning Robot - Sensor Test"
echo "================================================"
echo ""

cd ~/clean_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

echo "Starting multi-sensor bridge..."
echo "Sensors: 5x Ultrasonic, 8x IR (4 obj + 4 stair), 2x Encoder"
echo ""
echo "Topics to check:"
echo "  - /ultrasonic/* (5 sensors)"
echo "  - /ir/*/object (4 sensors)"
echo "  - /ir/*/stair (4 sensors)"
echo "  - /encoder/* (2 encoders)"
echo "  - /sensors/raw (debug)"
echo ""

ros2 run robot_sensors multi_sensor_node
