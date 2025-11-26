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
set -e
WORKSPACE="/home/saad/clean_ws"
MAP_NAME="${MAP_NAME:-room_map_$(date +%Y%m%d_%H%M%S)}"
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
    
    # Save the map (no timeout flag)
    ros2 run nav2_map_server map_saver_cli -f "$MAP_NAME" 2>&1 | tee "$LOG_DIR/map_saver.log"
    
    sleep 2
    
    print_info "Stopping all ROS2 nodes..."
    
    # Kill all processes
    pkill -9 -f multi_sensor_node || true
    pkill -9 -f sllidar_node || true
    pkill -9 -f motor_controller_node || true
    pkill -9 -f odometry_node || true
    pkill -9 -f slam_toolbox || true
    pkill -9 -f teleop_twist_keyboard || true
    pkill -9 -f static_transform_publisher || true
    
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
echo "   1. Start Arduino multi-sensor bridge (US/IR/IMU/encoders)"
echo "   2. Start LiDAR driver"
echo "   3. Start motor controller + odometry"
echo "   4. Start TFs and SLAM Toolbox"
echo "   5. Start keyboard teleop (arrow keys) to drive and build the map"
echo ""
print_warning "Make sure the robot has space to move and LiDAR/Arduino are plugged in."
echo ""
read -p "Press ENTER to start mapping session..."

# Source ROS2 workspace
print_info "Setting up ROS2 environment..."
cd "$WORKSPACE"
source install/setup.bash
print_success "ROS2 environment loaded"

# Kill any existing processes
print_info "Cleaning up old processes..."
pkill -f multi_sensor_node 2>/dev/null || true
pkill -f sllidar_node 2>/dev/null || true
pkill -f motor_controller_node 2>/dev/null || true
pkill -f odometry_node 2>/dev/null || true
pkill -f slam_toolbox 2>/dev/null || true
pkill -f teleop_twist_keyboard 2>/dev/null || true
pkill -f static_transform_publisher 2>/dev/null || true
sleep 3
print_success "Cleanup complete"

echo ""
print_status "🚀 STARTING COMPONENTS"

# 1. Multi-sensor bridge (Arduino: US/IR/IMU/encoders)
echo ""
print_info "[1/6] Starting Multi-Sensor Bridge..."
ros2 run robot_sensors multi_sensor_node > "$LOG_DIR/multi_sensor.log" 2>&1 &
MULTI_PID=$!
sleep 2
if ps -p $MULTI_PID > /dev/null; then
    print_success "Multi-Sensor Bridge started (PID: $MULTI_PID)"
else
    print_error "Multi-Sensor Bridge failed! Check $LOG_DIR/multi_sensor.log"
    exit 1
fi

# 2. LIDAR
echo ""
print_info "[2/6] Starting LiDAR (RPLIDAR A1)..."
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
    print_success "LiDAR started (PID: $LIDAR_PID)"
else
    print_error "LiDAR failed to start! Check $LOG_DIR/lidar.log"
    exit 1
fi

# 3. Motor controller + odometry
echo ""
print_info "[3/6] Starting Motor Controller..."
ros2 run robot_sensors motor_controller_node > "$LOG_DIR/motor_controller.log" 2>&1 &
MOTOR_PID=$!
sleep 2
if ps -p $MOTOR_PID > /dev/null; then
    print_success "Motor Controller started (PID: $MOTOR_PID)"
else
    print_error "Motor Controller failed! Check $LOG_DIR/motor_controller.log"
    cleanup
fi

echo ""
print_info "[3b/6] Starting Odometry..."
ros2 run robot_sensors odometry_node > "$LOG_DIR/odometry.log" 2>&1 &
ODOM_PID=$!
sleep 2
if ps -p $ODOM_PID > /dev/null; then
    print_success "Odometry started (PID: $ODOM_PID)"
else
    print_error "Odometry failed! Check $LOG_DIR/odometry.log"
    cleanup
fi

# 4. Static TFs (safety fallbacks)
echo ""
print_info "[4/6] Setting up coordinate transforms..."
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odom base_footprint \
    > "$LOG_DIR/tf_odom.log" 2>&1 &
ros2 run tf2_ros static_transform_publisher 0 0 0.05 0 0 0 base_footprint base_link \
    > "$LOG_DIR/tf_base.log" 2>&1 &
ros2 run tf2_ros static_transform_publisher 0.15 0 0.12 0 0 0 base_link laser \
    > "$LOG_DIR/tf_laser.log" 2>&1 &
sleep 1
print_success "TFs configured (odom -> base_footprint -> base_link -> laser)"

# 5. SLAM Toolbox
echo ""
print_info "[5/6] Starting SLAM Toolbox (online async)..."
ros2 launch slam_toolbox online_async_launch.py > "$LOG_DIR/slam.log" 2>&1 &
SLAM_PID=$!
sleep 5
if ps -p $SLAM_PID > /dev/null; then
    print_success "SLAM Toolbox started (PID: $SLAM_PID)"
else
    print_error "SLAM Toolbox failed! Check $LOG_DIR/slam.log"
    cleanup
fi

# 6. Keyboard teleop (foreground)
echo ""
print_status "✅ ALL SYSTEMS READY - START DRIVING"
echo "Controls: Arrow keys to drive, SPACE to stop, Ctrl+C to save map and quit."
echo ""
print_info "Teleop is running in the foreground. Build the map, then press Ctrl+C to save to ${MAP_NAME}.pgm/.yaml."

# This blocks; Ctrl+C triggers cleanup trap
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/cmd_vel

# If teleop exits normally, also save the map
cleanup
