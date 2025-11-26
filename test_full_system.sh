#!/bin/bash

# Full System Test Script
# Tests all components to ensure they're working

echo "🧪 FULL SYSTEM TEST"
echo "===================="
echo ""

cd /home/saad/clean_ws
source install/setup.bash

# Test 1: Check if all nodes are running
echo "1️⃣ Checking ROS Nodes..."
nodes=$(ros2 node list 2>/dev/null | wc -l)
if [ $nodes -ge 6 ]; then
    echo "   ✅ Found $nodes nodes running"
else
    echo "   ❌ Expected at least 6 nodes, found $nodes"
fi
echo ""

# Test 2: Check LiDAR data
echo "2️⃣ Testing LiDAR (/scan)..."
timeout 2 bash -c 'ros2 topic hz /scan 2>&1 | grep "average rate"' > /tmp/test_lidar.txt 2>&1 &
sleep 3
if grep -q "average rate" /tmp/test_lidar.txt 2>/dev/null; then
    rate=$(grep "average rate" /tmp/test_lidar.txt | awk '{print $3}')
    echo "   ✅ LiDAR publishing at ${rate} Hz"
else
    echo "   ❌ LiDAR not publishing data"
fi
echo ""

# Test 3: Check ultrasonic sensors
echo "3️⃣ Testing Ultrasonic Sensors..."
us_topics=("/ultrasonic/front" "/ultrasonic/front_left" "/ultrasonic/front_right" "/ultrasonic/right")
for topic in "${us_topics[@]}"; do
    if timeout 1 ros2 topic echo $topic --once >/dev/null 2>&1; then
        echo "   ✅ $topic: OK"
    else
        echo "   ⚠️  $topic: No data"
    fi
done
echo ""

# Test 4: Check battery
echo "4️⃣ Testing Battery (/battery/state)..."
if timeout 2 ros2 topic echo /battery/state --once >/dev/null 2>&1; then
    echo "   ✅ Battery data available"
else
    echo "   ❌ Battery data not available"
fi
echo ""

# Test 5: Check odometry
echo "5️⃣ Testing Odometry (/odom)..."
if timeout 2 ros2 topic echo /odom --once >/dev/null 2>&1; then
    echo "   ✅ Odometry data available"
else
    echo "   ❌ Odometry data not available"
fi
echo ""

# Test 6: Check rosbridge
echo "6️⃣ Testing Rosbridge (port 9090)..."
if lsof -i :9090 >/dev/null 2>&1; then
    echo "   ✅ Rosbridge running on port 9090"
else
    echo "   ❌ Rosbridge not running"
fi
echo ""

# Test 7: Check topic list
echo "7️⃣ Key Topics Check..."
required_topics=(
    "/scan"
    "/odom"
    "/cmd_vel"
    "/battery/state"
    "/ultrasonic/front"
    "/relay/command"
    "/autonomous/enable"
)

for topic in "${required_topics[@]}"; do
    if ros2 topic list 2>/dev/null | grep -q "^${topic}$"; then
        echo "   ✅ $topic"
    else
        echo "   ❌ $topic missing"
    fi
done
echo ""

# Test 8: Dashboard connection
echo "8️⃣ Testing Dashboard..."
if curl -s http://localhost:3000 >/dev/null 2>&1; then
    echo "   ✅ Dashboard accessible at http://localhost:3000"
else
    echo "   ⚠️  Dashboard not responding (may need to start)"
fi
echo ""

echo "===================="
echo "✅ System Test Complete!"
echo ""
echo "📊 Quick Check Commands:"
echo "   - View LiDAR: ros2 topic echo /scan"
echo "   - Check nodes: ros2 node list"
echo "   - Monitor logs: tail -f /tmp/industry_avoidance.log"
echo "   - Dashboard: http://localhost:3000"
echo ""
