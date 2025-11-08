#!/bin/bash
# Industry-Grade Floor Cleaning Robot - Topic Verification
# Checks all expected ROS2 topics are publishing

echo "================================================"
echo "  Floor Cleaning Robot - Topic Verification"
echo "================================================"
echo ""

cd ~/clean_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

echo "Expected Topics (Industry-Grade Configuration):"
echo ""
echo "📡 SENSORS:"
echo "  Ultrasonic (5):"
echo "    - /ultrasonic/front"
echo "    - /ultrasonic/front_right"
echo "    - /ultrasonic/front_left"
echo "    - /ultrasonic/right"
echo "    - /ultrasonic/left"
echo ""
echo "  IR Object Detection (4):"
echo "    - /ir/front_right/object"
echo "    - /ir/front_left/object"
echo "    - /ir/back_right/object"
echo "    - /ir/back_left/object"
echo ""
echo "  IR Stair Detection (4):"
echo "    - /ir/front_right/stair"
echo "    - /ir/front_left/stair"
echo "    - /ir/back_right/stair"
echo "    - /ir/back_left/stair"
echo ""
echo "  Encoders (2):"
echo "    - /encoder/left"
echo "    - /encoder/right"
echo ""
echo "  LIDAR (1):"
echo "    - /scan"
echo ""
echo "🤖 NAVIGATION:"
echo "    - /odom (Odometry - position & velocity)"
echo "    - /cmd_vel (Velocity commands - input)"
echo ""
echo "🔧 DEBUG:"
echo "    - /sensors/raw (Raw sensor data)"
echo "    - /motor/status (Motor status)"
echo ""
echo "📊 SYSTEM:"
echo "    - /parameter_events"
echo "    - /rosout"
echo "    - /tf (Transform tree)"
echo "    - /tf_static (Static transforms)"
echo ""
echo "Total Expected: ~25 topics"
echo ""
echo "================================================"
echo "Checking active topics..."
echo "================================================"
echo ""

# Get topic list
TOPICS=$(ros2 topic list 2>/dev/null)
TOPIC_COUNT=$(echo "$TOPICS" | wc -l)

echo "Active Topics ($TOPIC_COUNT):"
echo "$TOPICS"
echo ""

# Check critical topics
echo "================================================"
echo "Verifying Critical Topics:"
echo "================================================"
echo ""

check_topic() {
    topic=$1
    if echo "$TOPICS" | grep -q "^${topic}$"; then
        echo "✅ $topic"
        return 0
    else
        echo "❌ $topic - MISSING!"
        return 1
    fi
}

# Sensors
check_topic "/scan"
check_topic "/ultrasonic/front"
check_topic "/encoder/left"
check_topic "/encoder/right"

# Navigation
check_topic "/odom"
check_topic "/cmd_vel"

# Debug
check_topic "/sensors/raw"

echo ""
echo "================================================"
echo "Topic Type Check:"
echo "================================================"
echo ""

ros2 topic list -t | grep -E "(scan|odom|ultrasonic|encoder|cmd_vel)" | head -10

echo ""
echo "================================================"
echo "To monitor a specific topic:"
echo "  ros2 topic echo /odom"
echo "  ros2 topic echo /scan"
echo "  ros2 topic hz /odom"
echo "================================================"
