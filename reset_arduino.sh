#!/bin/bash
# Quick Arduino Reset Script
# Resets the Arduino by toggling DTR on serial port

echo "╔════════════════════════════════════════════════════════════╗"
echo "║           🔄 ARDUINO RESET 🔄                              ║"
echo "╚════════════════════════════════════════════════════════════╝"
echo ""

# Check if Arduino is connected
if [ ! -e /dev/ttyACM0 ]; then
    echo "❌ Arduino not found on /dev/ttyACM0"
    exit 1
fi

echo "🛑 Stopping ROS nodes..."
pkill -f multi_sensor_node
pkill -f motor_controller_node
sleep 1

echo "🔄 Resetting Arduino..."

# Reset Arduino by toggling DTR
python3 - <<EOF
import serial
import time

try:
    ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
    ser.setDTR(False)
    time.sleep(0.1)
    ser.setDTR(True)
    time.sleep(2)
    ser.close()
    print("✅ Arduino reset successful!")
except Exception as e:
    print(f"❌ Error: {e}")
    exit(1)
EOF

if [ $? -eq 0 ]; then
    echo ""
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo "✅ Arduino has been reset!"
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo ""
    echo "Next: Restart ROS system"
    echo "  ./start_industry_grade.sh"
    echo ""
else
    echo "❌ Reset failed. Try manually:"
    echo "1. Press the RESET button on Arduino"
    echo "2. Or unplug and replug USB cable"
fi
