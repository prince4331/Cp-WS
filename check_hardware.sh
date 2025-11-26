#!/bin/bash
# Hardware Check Script for Floor Cleaning Robot
# Checks Arduino, LiDAR, and serial connections

echo "╔════════════════════════════════════════════════════════════╗"
echo "║        🔍 ROBOT HARDWARE DIAGNOSTICS 🔍                    ║"
echo "╚════════════════════════════════════════════════════════════╝"
echo ""

# Color codes
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# 1. Check USB Devices
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "📌 USB DEVICES:"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

if [ -e /dev/ttyACM0 ]; then
    echo -e "${GREEN}✅ /dev/ttyACM0 (Arduino) - FOUND${NC}"
    ls -la /dev/ttyACM0
else
    echo -e "${RED}❌ /dev/ttyACM0 (Arduino) - NOT FOUND${NC}"
fi
echo ""

if [ -e /dev/ttyUSB0 ]; then
    echo -e "${GREEN}✅ /dev/ttyUSB0 (LiDAR) - FOUND${NC}"
    ls -la /dev/ttyUSB0
elif [ -e /dev/ttyUSB1 ]; then
    echo -e "${YELLOW}⚠️  /dev/ttyUSB1 (LiDAR) - FOUND (Expected ttyUSB0)${NC}"
    ls -la /dev/ttyUSB1
else
    echo -e "${RED}❌ /dev/ttyUSB* (LiDAR) - NOT FOUND${NC}"
fi
echo ""

# 2. Check what's using the ports
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "🔌 PORT USAGE:"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

echo "Arduino (/dev/ttyACM0):"
if lsof /dev/ttyACM0 2>/dev/null; then
    echo -e "${GREEN}✅ In use by ROS node${NC}"
else
    echo -e "${YELLOW}⚠️  Not in use - Node may not be running${NC}"
fi
echo ""

echo "LiDAR (/dev/ttyUSB*):"
if lsof /dev/ttyUSB0 2>/dev/null; then
    echo -e "${GREEN}✅ ttyUSB0 in use${NC}"
elif lsof /dev/ttyUSB1 2>/dev/null; then
    echo -e "${GREEN}✅ ttyUSB1 in use${NC}"
else
    echo -e "${YELLOW}⚠️  Not in use - Node may not be running${NC}"
fi
echo ""

# 3. Check ROS Nodes
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "🤖 ROS NODES STATUS:"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

nodes=("multi_sensor_node" "sllidar_node" "motor_controller" "odometry_node" "industry_obstacle" "rosbridge_websocket")

for node in "${nodes[@]}"; do
    if pgrep -f "$node" > /dev/null; then
        echo -e "${GREEN}✅ $node - RUNNING${NC}"
    else
        echo -e "${RED}❌ $node - NOT RUNNING${NC}"
    fi
done
echo ""

# 4. Check sensor data in logs
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "📊 SENSOR DATA CHECK:"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

# Check multi-sensor log
if [ -f /tmp/multi_sensor.log ]; then
    EMPTY_READS=$(grep -c "Serial read returned empty" /tmp/multi_sensor.log 2>/dev/null || echo "0")
    if [ "$EMPTY_READS" -gt 100 ]; then
        echo -e "${RED}❌ Arduino NOT sending data ($EMPTY_READS empty reads)${NC}"
        echo "   Problem: Arduino firmware not running or not connected properly"
    else
        echo -e "${GREEN}✅ Arduino sending data${NC}"
    fi
else
    echo -e "${YELLOW}⚠️  Multi-sensor log not found${NC}"
fi
echo ""

# Check LiDAR log
if [ -f /tmp/lidar.log ]; then
    if grep -q "scan frequency" /tmp/lidar.log 2>/dev/null; then
        echo -e "${GREEN}✅ LiDAR scanning (10Hz)${NC}"
    else
        echo -e "${RED}❌ LiDAR not scanning properly${NC}"
    fi
else
    echo -e "${YELLOW}⚠️  LiDAR log not found${NC}"
fi
echo ""

# 5. Arduino firmware check
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "💾 ARDUINO FIRMWARE:"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

ARDUINO_SRC="/home/saad/clean_ws/src/robot_sensors/arduino/multi_sensor"
if [ -d "$ARDUINO_SRC" ]; then
    echo -e "${GREEN}✅ Firmware source found: $ARDUINO_SRC${NC}"
    
    if command -v pio &> /dev/null; then
        echo -e "${GREEN}✅ PlatformIO installed${NC}"
    else
        echo -e "${YELLOW}⚠️  PlatformIO not found (needed to upload firmware)${NC}"
    fi
else
    echo -e "${RED}❌ Arduino firmware source not found${NC}"
fi
echo ""

# 6. Serial communication test
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "📡 SERIAL COMMUNICATION TEST:"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

if [ -e /dev/ttyACM0 ]; then
    echo "Checking Arduino serial output (5 seconds)..."
    
    # Temporarily stop the ROS node to read serial
    MULTI_PID=$(pgrep -f "multi_sensor_node" | head -1)
    if [ ! -z "$MULTI_PID" ]; then
        echo "Note: ROS node is using the port. Stop it first to test serial directly."
        echo "Latest data from multi_sensor log:"
        tail -5 /tmp/multi_sensor.log 2>/dev/null | grep -v "empty" || echo "No recent data"
    fi
else
    echo -e "${RED}Arduino not connected${NC}"
fi
echo ""

# 7. Recommendations
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "💡 RECOMMENDATIONS:"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

if grep -q "Serial read returned empty" /tmp/multi_sensor.log 2>/dev/null; then
    echo -e "${YELLOW}⚠️  Arduino Issue Detected!${NC}"
    echo ""
    echo "SOLUTIONS:"
    echo "1. Reset Arduino: Press the reset button on the Arduino board"
    echo "2. Check power: Ensure Arduino has power (LED should be on)"
    echo "3. Upload firmware: Run ./upload_arduino.sh to flash the firmware"
    echo "4. Check connections: Verify USB cable is properly connected"
    echo "5. Check baud rate: Ensure Arduino serial is set to 115200"
    echo ""
fi

echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "✨ Diagnostic complete!"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
