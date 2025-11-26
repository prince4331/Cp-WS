#!/bin/bash
# Arduino Auto-Reset Script
# Automatically resets Arduino when it stops sending data

echo "╔════════════════════════════════════════════════════════════╗"
echo "║     🔄 ARDUINO AUTO-RESET MONITOR 🔄                       ║"
echo "╚════════════════════════════════════════════════════════════╝"
echo ""
echo "Monitoring Arduino data flow..."
echo "Will auto-reset if no sensor data for 30 seconds"
echo "Press Ctrl+C to stop"
echo ""

CHECK_INTERVAL=10  # Check every 10 seconds
MAX_EMPTY_COUNT=300  # Reset if empty count exceeds this

while true; do
    sleep $CHECK_INTERVAL
    
    # Check how many "empty" reads in the log
    EMPTY_COUNT=$(grep -c "Serial read returned empty" /tmp/multi_sensor.log 2>/dev/null || echo "0")
    
    # Check if ultrasonic data was published recently
    RECENT_DATA=$(grep "Published.*ultrasonic_front" /tmp/multi_sensor.log 2>/dev/null | tail -1)
    
    if [ -z "$RECENT_DATA" ]; then
        LAST_TIME="never"
    else
        LAST_TIME=$(echo "$RECENT_DATA" | grep -oP '\[\d+\.\d+\]' | tail -1)
    fi
    
    if [ "$EMPTY_COUNT" -gt "$MAX_EMPTY_COUNT" ]; then
        echo "⚠️  Arduino stopped sending data (empty count: $EMPTY_COUNT)"
        echo "🔄 Auto-resetting Arduino..."
        
        cd /home/saad/clean_ws
        ./reset_arduino.sh > /dev/null 2>&1
        
        sleep 3
        
        # Restart ROS nodes
        echo "🔄 Restarting ROS nodes..."
        ./start_industry_grade.sh > /dev/null 2>&1
        
        echo "✅ Arduino reset complete at $(date)"
        echo ""
    else
        echo "✅ Arduino OK (empty: $EMPTY_COUNT, last data: $LAST_TIME)"
    fi
done
