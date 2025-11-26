#!/bin/bash
# Diagnose Stuck Ultrasonic Sensors

echo "======================================"
echo "🔍 STUCK SENSOR DIAGNOSTIC"
echo "======================================"
echo ""
echo "Checking if sensors are truly STUCK..."
echo ""

echo "📊 Collecting 10 readings..."
for i in {1..10}; do
    reading=$(tail -1 /tmp/multi_sensor.log | grep -oP 'US:\K[^|]+' 2>/dev/null)
    if [ ! -z "$reading" ]; then
        echo "Reading $i: $reading"
    fi
    sleep 0.5
done

echo ""
echo "======================================"
echo "�� ANALYSIS:"
echo "======================================"
echo ""
echo "✅ WORKING SENSORS = Values change (55.34, 55.88, 56.12, etc.)"
echo "❌ STUCK SENSORS = Same value every time (5.00, 5.00, 5.00)"
echo ""
echo "If Front-Left and Left always show EXACTLY same value:"
echo "  → HARDWARE PROBLEM (blocked, broken, or wiring issue)"
echo ""
echo "�� NEXT STEPS:"
echo "1. Power OFF robot"
echo "2. Inspect Front-Left sensor (Pins 36-Trig/37-Echo)"
echo "3. Inspect Left sensor (Pins 32-Trig/33-Echo)"
echo "4. Check for:"
echo "   - Physical obstructions"
echo "   - Sensor angle (should point outward)"
echo "   - Loose wiring"
echo "   - Damaged sensor"
echo "======================================"
