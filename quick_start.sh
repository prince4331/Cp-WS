#!/bin/bash

# Quick Start Guide for Floor Cleaning Robot

echo "╔════════════════════════════════════════════════════════════════╗"
echo "║         🤖 FLOOR CLEANING ROBOT - QUICK START GUIDE          ║"
echo "╚════════════════════════════════════════════════════════════════╝"
echo ""

echo "📋 AVAILABLE COMMANDS:"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""

echo "1️⃣  START COMPLETE ROBOT SYSTEM"
echo "   Terminal 1:"
echo "   cd /home/saad/clean_ws"
echo "   source install/setup.bash"
echo "   ros2 launch robot_sensors complete_robot_launch.py"
echo ""

echo "2️⃣  OPEN SENSOR DASHBOARD (in new terminal)"
echo "   Terminal 2:"
echo "   cd /home/saad/clean_ws"
echo "   ./dashboard.sh"
echo ""

echo "3️⃣  CHECK SYSTEM STATUS"
echo "   ./show_status.sh"
echo ""

echo "4️⃣  VERIFY ALL TOPICS"
echo "   ros2 topic list"
echo "   ros2 topic hz /ultrasonic/front  # Check update rate"
echo ""

echo "5️⃣  TEST ROBOT MOVEMENT"
echo "   # Move forward"
echo "   ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \"{linear: {x: 0.1}}\" --once"
echo ""
echo "   # Stop"
echo "   ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \"{}\" --once"
echo ""

echo "6️⃣  UPLOAD NEW ARDUINO CODE"
echo "   cd /home/saad/clean_ws/src/robot_sensors/arduino/multi_sensor"
echo "   pio run --target upload"
echo ""

echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""
echo "📚 DOCUMENTATION FILES:"
echo "   • README.md              - Main project documentation"
echo "   • DASHBOARD_README.md    - Dashboard usage guide"
echo "   • SYSTEM_COMPLETE.md     - System architecture"
echo "   • SENSORS_README.md      - Sensor pinout & wiring"
echo ""

echo "🔌 HARDWARE CONNECTIONS:"
echo "   • Arduino Mega: /dev/ttyACM0 (115200 baud)"
echo "   • SLLIDAR A1:   /dev/ttyUSB0 (115200 baud)"
echo ""

echo "📊 SENSOR ARRAY:"
echo "   • 5x HC-SR04 Ultrasonic sensors"
echo "   • 4x IR Object Detection sensors"
echo "   • 4x IR Stair Detection sensors"
echo "   • 2x Wheel Encoders (800 ticks/rev)"
echo "   • 1x SLLIDAR A1 (360° LIDAR)"
echo ""

echo "✅ VERIFY SYSTEM:"
echo "   Expected topic count: 24 topics"
echo "   Expected node count:  5 nodes"
echo ""

echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""
echo "For detailed help, see: /home/saad/clean_ws/DASHBOARD_README.md"
echo ""
