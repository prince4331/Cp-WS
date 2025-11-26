#!/bin/bash
# Arduino Firmware Upload Script
# Uploads the multi-sensor firmware to Arduino

echo "╔════════════════════════════════════════════════════════════╗"
echo "║         📤 ARDUINO FIRMWARE UPLOAD 📤                      ║"
echo "╚════════════════════════════════════════════════════════════╝"
echo ""

# Color codes
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

ARDUINO_DIR="/home/saad/clean_ws/src/robot_sensors/arduino/multi_sensor"

# Check if Arduino is connected
if [ ! -e /dev/ttyACM0 ]; then
    echo -e "${RED}❌ Arduino not found on /dev/ttyACM0${NC}"
    echo "Please connect the Arduino and try again."
    exit 1
fi

echo -e "${GREEN}✅ Arduino found on /dev/ttyACM0${NC}"
echo ""

# Check if firmware directory exists
if [ ! -d "$ARDUINO_DIR" ]; then
    echo -e "${RED}❌ Firmware directory not found: $ARDUINO_DIR${NC}"
    exit 1
fi

echo -e "${GREEN}✅ Firmware directory found${NC}"
echo ""

# Check if PlatformIO is installed
if ! command -v pio &> /dev/null; then
    echo -e "${YELLOW}⚠️  PlatformIO not found. Installing...${NC}"
    echo ""
    
    # Install PlatformIO
    python3 -c "$(curl -fsSL https://raw.githubusercontent.com/platformio/platformio/master/scripts/get-platformio.py)"
    
    if [ $? -ne 0 ]; then
        echo -e "${RED}❌ Failed to install PlatformIO${NC}"
        echo ""
        echo "Manual installation:"
        echo "  pip3 install platformio"
        exit 1
    fi
fi

echo -e "${GREEN}✅ PlatformIO ready${NC}"
echo ""

# Stop ROS nodes using the Arduino
echo "🛑 Stopping ROS nodes that use Arduino..."
pkill -f multi_sensor_node
pkill -f motor_controller_node
sleep 2

echo -e "${GREEN}✅ ROS nodes stopped${NC}"
echo ""

# Navigate to Arduino directory
cd "$ARDUINO_DIR" || exit 1

echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "📤 Uploading firmware to Arduino..."
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""

# Upload firmware
pio run --target upload

if [ $? -eq 0 ]; then
    echo ""
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo -e "${GREEN}✅ ✅ ✅ FIRMWARE UPLOAD SUCCESSFUL! ✅ ✅ ✅${NC}"
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo ""
    echo "Next steps:"
    echo "1. Wait 2 seconds for Arduino to initialize"
    sleep 2
    echo "2. Restart ROS system: ./start_industry_grade.sh"
    echo ""
else
    echo ""
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo -e "${RED}❌ FIRMWARE UPLOAD FAILED${NC}"
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo ""
    echo "Troubleshooting:"
    echo "1. Check USB connection"
    echo "2. Press Arduino reset button and try again"
    echo "3. Check permissions: sudo chmod 666 /dev/ttyACM0"
    echo "4. Verify Arduino board type in platformio.ini"
    echo ""
    exit 1
fi
