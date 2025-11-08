#!/bin/bash

# LIDAR Fix Script - Sets scan mode after node starts

echo "⏳ Waiting for LIDAR node to start..."
sleep 5

echo "🔧 Setting LIDAR scan mode..."
ros2 param set /sllidar_node scan_mode "Sensitivity"

echo "✅ LIDAR should now be publishing on /scan"
echo ""
echo "Verify with: ros2 topic hz /scan"
