#!/bin/bash

# Robot Dashboard Launcher
# This script sets up the environment and starts the web dashboard

echo "================================================"
echo "  ROBOT DASHBOARD LAUNCHER"
echo "================================================"
echo ""

# Source ROS2 environment
echo "[1/4] Sourcing ROS2 Humble environment..."
source /opt/ros/humble/setup.bash

# Source workspace
echo "[2/4] Sourcing workspace..."
cd /home/saad/clean_ws
source install/setup.bash

# Check if Flask is installed
echo "[3/4] Checking Flask installation..."
if ! python3 -c "import flask" 2>/dev/null; then
    echo "Flask not found. Installing Flask..."
    pip3 install flask
else
    echo "Flask is already installed."
fi

# Start the dashboard
echo "[4/4] Starting Robot Dashboard..."
echo ""
echo "================================================"
echo "  Dashboard will be available at:"
echo "  http://localhost:5000"
echo "  or"
echo "  http://$(hostname -I | awk '{print $1}'):5000"
echo "================================================"
echo ""
echo "Press Ctrl+C to stop the dashboard"
echo ""

python3 /home/saad/clean_ws/robot_dashboard.py
