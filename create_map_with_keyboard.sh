#!/bin/bash

################################################################################
# SLAM Mapping Script with Keyboard Control
# 
# This script starts all components needed to create a map of your room:
# - LIDAR sensor for environment scanning
# - Motor controller for robot movement
# - Odometry for position tracking
# - SLAM Toolbox for real-time map building
# - Foxglove Bridge for web visualization
# - Keyboard teleop for manual control with arrow keys
#
# Usage: bash create_map_with_keyboard.sh
# 
# Controls:
#   Arrow Keys: Move robot (Up=Forward, Down=Backward, Left=Turn Left, Right=Turn Right)
#   Space: Stop
#   Ctrl+C: Stop mapping and save map
#
################################################################################

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
MAGENTA='\033[0;35m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# Configuration
WORKSPACE="/home/saad/clean_ws"
MAP_NAME="my_room_map"
LOG_DIR="/tmp/slam_mapping_logs"

# Create log directory
mkdir -p "$LOG_DIR"

# Function to print colored messages
print_status() {
    echo -e "${CYAN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo -e "${GREEN}$1${NC}"
    echo -e "${CYAN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
}

print_info() {
    echo -e "${BLUE}ℹ️  $1${NC}"
}

print_success() {
    echo -e "${GREEN}✅ $1${NC}"
}

print_warning() {
    echo -e "${YELLOW}⚠️  $1${NC}"
}

print_error() {
    echo -e "${RED}❌ $1${NC}"
}

# Cleanup function
cleanup() {
    echo ""
    print_status "🛑 STOPPING MAPPING SESSION"
    
    print_info "Saving map to: ${MAP_NAME}.pgm and ${MAP_NAME}.yaml"
    cd "$WORKSPACE"
    source install/setup.bash
    
    # Save the map (without timeout parameter - it's deprecated)
    ros2 run nav2_map_server map_saver_cli -f "$MAP_NAME" 2>&1 | tee "$LOG_DIR/map_saver.log"
    
    sleep 2
    
    print_info "Stopping all ROS2 nodes..."
    
    # Kill all processes
    pkill -9 -f sllidar_node
    pkill -9 -f motor_controller_node
    pkill -9 -f odometry_publisher
    pkill -9 -f slam_toolbox
    pkill -9 -f foxglove_bridge
    pkill -9 -f teleop_twist_keyboard
    pkill -9 -f static_transform_publisher
    
    sleep 2
    
    print_status "📊 MAPPING SESSION COMPLETE!"
    
    if [ -f "${WORKSPACE}/${MAP_NAME}.pgm" ] && [ -f "${WORKSPACE}/${MAP_NAME}.yaml" ]; then
        print_success "Map saved successfully!"
        echo ""
        echo "📁 Map files created:"
        ls -lh "${WORKSPACE}/${MAP_NAME}".*
        echo ""
        print_info "You can now use this map for navigation!"
    else
        print_warning "Map files not found. Check logs in $LOG_DIR"
    fi
    
    exit 0
}

# Trap Ctrl+C to save map before exiting
trap cleanup SIGINT SIGTERM

# Main script
clear
print_status "🗺️  SLAM MAPPING WITH KEYBOARD CONTROL"

echo ""
print_info "This script will:"
echo "   1. Start LIDAR sensor"
echo "   2. Start motor controller"
echo "   3. Start odometry tracking"
echo "   4. Start SLAM for real-time mapping"
echo "   5. Start Foxglove for web visualization"
echo "   6. Enable keyboard control (arrow keys)"
echo ""
print_warning "Make sure your robot has enough space to move!"
echo ""
read -p "Press ENTER to start mapping session..."

# Source ROS2 workspace
print_info "Setting up ROS2 environment..."
cd "$WORKSPACE"
source install/setup.bash
print_success "ROS2 environment loaded"

# Kill any existing processes
print_info "Cleaning up old processes..."
pkill -f sllidar_node 2>/dev/null
pkill -f motor_controller_node 2>/dev/null
pkill -f odometry_publisher 2>/dev/null
pkill -f slam_toolbox 2>/dev/null
pkill -f foxglove_bridge 2>/dev/null
pkill -f teleop_twist_keyboard 2>/dev/null
pkill -f static_transform_publisher 2>/dev/null
sleep 3
print_success "Cleanup complete"

echo ""
print_status "🚀 STARTING COMPONENTS"

# 1. Start LIDAR
echo ""
print_info "[1/7] Starting LIDAR (RPLIDAR A1)..."
ros2 run sllidar_ros2 sllidar_node --ros-args \
    -p serial_port:=/dev/ttyUSB0 \
    -p serial_baudrate:=115200 \
    -p frame_id:=laser \
    -p angle_compensate:=true \
    -p scan_mode:=Standard \
    > "$LOG_DIR/lidar.log" 2>&1 &
LIDAR_PID=$!
sleep 3
if ps -p $LIDAR_PID > /dev/null; then
    print_success "LIDAR started (PID: $LIDAR_PID)"
else
    print_error "LIDAR failed to start! Check $LOG_DIR/lidar.log"
    exit 1
fi

# 2. Start Motor Controller
echo ""
print_info "[2/7] Starting Motor Controller..."
ros2 run robot_sensors motor_controller_node \
    > "$LOG_DIR/motor_controller.log" 2>&1 &
MOTOR_PID=$!
sleep 2
if ps -p $MOTOR_PID > /dev/null; then
    print_success "Motor Controller started (PID: $MOTOR_PID)"
else
    print_error "Motor Controller failed to start! Check $LOG_DIR/motor_controller.log"
    cleanup
fi

# 3. Start Odometry
echo ""
print_info "[3/7] Starting Odometry Publisher..."
ros2 run robot_sensors odometry_publisher \
    > "$LOG_DIR/odometry.log" 2>&1 &
ODOM_PID=$!
sleep 2
if ps -p $ODOM_PID > /dev/null; then
    print_success "Odometry started (PID: $ODOM_PID)"
else
    print_error "Odometry failed to start! Check $LOG_DIR/odometry.log"
    cleanup
fi

# 4. Start TF transforms
echo ""
print_info "[4/7] Setting up coordinate transforms..."

# odom -> base_footprint (odometry provides this dynamically, but we need a static fallback)
# This will be overridden by odometry when it starts publishing
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odom base_footprint \
    > "$LOG_DIR/tf_odom.log" 2>&1 &
TF0_PID=$!

# base_footprint -> base_link (robot base)
ros2 run tf2_ros static_transform_publisher 0 0 0.05 0 0 0 base_footprint base_link \
    > "$LOG_DIR/tf_base.log" 2>&1 &
TF1_PID=$!

# base_link -> laser (LIDAR position)
ros2 run tf2_ros static_transform_publisher 0.15 0 0.12 0 0 0 base_link laser \
    > "$LOG_DIR/tf_laser.log" 2>&1 &
TF2_PID=$!

sleep 1
print_success "Coordinate transforms configured (odom -> base_footprint -> base_link -> laser)"

# 5. Start SLAM Toolbox
echo ""
print_info "[5/7] Starting SLAM Toolbox (Real-time Mapping)..."
ros2 launch slam_toolbox online_async_launch.py \
    > "$LOG_DIR/slam.log" 2>&1 &
SLAM_PID=$!
sleep 5
if ps -p $SLAM_PID > /dev/null; then
    print_success "SLAM Toolbox started (PID: $SLAM_PID)"
else
    print_error "SLAM Toolbox failed to start! Check $LOG_DIR/slam.log"
    cleanup
fi

# 6. Start Foxglove Bridge
echo ""
print_info "[6/7] Starting Foxglove Bridge (Web Visualization)..."
ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765 \
    > "$LOG_DIR/foxglove.log" 2>&1 &
FOXGLOVE_PID=$!
sleep 2
if ps -p $FOXGLOVE_PID > /dev/null; then
    print_success "Foxglove Bridge started (PID: $FOXGLOVE_PID)"
else
    print_error "Foxglove Bridge failed to start! Check $LOG_DIR/foxglove.log"
    cleanup
fi

# 7. Verify topics
echo ""
print_info "[7/7] Verifying ROS2 topics..."
sleep 2
TOPICS=$(timeout 3 ros2 topic list 2>/dev/null | grep -E "(scan|odom|map|cmd_vel)" || echo "")
if echo "$TOPICS" | grep -q "scan" && echo "$TOPICS" | grep -q "odom" && echo "$TOPICS" | grep -q "map"; then
    print_success "All critical topics detected!"
else
    print_warning "Some topics might be missing. Check logs."
fi

# Get IP address
IP_ADDRESS=$(hostname -I | awk '{print $1}')

# Display status
echo ""
print_status "✅ ALL SYSTEMS READY!"

cat << EOF

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
📡 SYSTEM STATUS
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

✅ LIDAR:            Publishing /scan
✅ Motor Controller:  Ready for /cmd_vel commands
✅ Odometry:         Publishing /odom
✅ SLAM Toolbox:     Building map in real-time (/map)
✅ Foxglove Bridge:  Running on port 8765
✅ TF Transforms:    Coordinate system configured

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
🌐 FOXGLOVE WEB VISUALIZATION
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

1. Open browser: https://app.foxglove.dev/
2. Click: "Open Connection"
3. Select: "Foxglove WebSocket"
4. Enter: ws://${IP_ADDRESS}:8765
5. Add "3D Panel" and enable:
   - /scan (LaserScan)
   - /map (OccupancyGrid)

You will see:
• Real-time LIDAR scans
• Live map building as you drive
• Robot position and orientation

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
⌨️  KEYBOARD CONTROL
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

Starting keyboard teleop in 3 seconds...

Controls:
   ⬆️  Arrow UP    = Move Forward
   ⬇️  Arrow DOWN  = Move Backward
   ⬅️  Arrow LEFT  = Turn Left
   ➡️  Arrow RIGHT = Turn Right
   SPACE          = Stop
   q              = Quit (will save map)

Tips for mapping:
• Drive slowly for best results
• Cover entire room systematically
• Go close to walls for accurate boundaries
• Revisit areas to improve loop closure
• Watch Foxglove to see map building

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
💾 SAVING MAP
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

When finished mapping:
   1. Press Ctrl+C in this terminal
   2. Map will be automatically saved as:
      - ${MAP_NAME}.pgm  (image)
      - ${MAP_NAME}.yaml (metadata)

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

EOF

echo -e "${YELLOW}Starting keyboard control in:${NC}"
for i in 3 2 1; do
    echo -e "${YELLOW}   $i...${NC}"
    sleep 1
done

echo ""
print_status "🎮 KEYBOARD CONTROL ACTIVE"
echo ""
print_warning "Move your robot around the room to create the map!"
print_warning "Press Ctrl+C when finished to save the map."
echo ""

# Start keyboard teleop (this will block until user quits)
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/cmd_vel

# If user quits teleop normally (with 'q'), also save map
cleanup
