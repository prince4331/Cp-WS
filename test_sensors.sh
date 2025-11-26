#!/bin/bash
# Quick Sensor Test Script

echo "======================================"
echo "�� ROBOT SENSOR QUICK TEST"
echo "======================================"
echo ""

echo "📡 Current Sensor Readings:"
echo "----------------------------"

# Get one reading from each sensor
echo "Front:       $(ros2 topic echo /ultrasonic/front --once 2>/dev/null | grep 'range:' | awk '{print $2*100 "cm"}')"
echo "Front-Right: $(ros2 topic echo /ultrasonic/front_right --once 2>/dev/null | grep 'range:' | awk '{print $2*100 "cm"}')"
echo "Front-Left:  $(ros2 topic echo /ultrasonic/front_left --once 2>/dev/null | grep 'range:' | awk '{print $2*100 "cm"}') ⚠️"
echo "Right:       $(ros2 topic echo /ultrasonic/right --once 2>/dev/null | grep 'range:' | awk '{print $2*100 "cm"}')"
echo "Left:        $(ros2 topic echo /ultrasonic/left --once 2>/dev/null | grep 'range:' | awk '{print $2*100 "cm"}') ⚠️"

echo ""
echo "💧 Water Tank Levels:"
echo "----------------------------"
echo "Clean Tank:  $(ros2 topic echo /water_level/clean --once 2>/dev/null | grep 'range:' | awk '{print $2*100 "cm"}') (3-5cm=full, 15-20cm=empty)"
echo "Dirty Tank:  $(ros2 topic echo /water_level/dirty --once 2>/dev/null | grep 'range:' | awk '{print $2*100 "cm"}')"

echo ""
echo "✅ If front-left and left show <10cm in OPEN SPACE:"
echo "   → Check sensor mounting/angle"
echo "   → Sensor may point at robot body"
echo ""
echo "✅ Clean tank 3-5cm = FULL (normal!)"
echo "======================================"
