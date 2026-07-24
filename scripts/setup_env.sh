#!/usr/bin/env bash
set -euo pipefail

WORKSPACE_DIR="$(cd "$(dirname "$0")/.." && pwd)"

echo "=== ZENS-Final-AUVC Environment Setup ==="

# Source ROS2
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
elif [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
else
    echo "Could not find ROS2 setup.bash. Is ROS2 installed?"
    exit 1
fi

# Build the workspace
echo "Building workspace..."
cd "$WORKSPACE_DIR/ros_ws"
colcon build --symlink-install

# Source the workspace
source install/setup.bash

echo ""
echo "=== Build complete ==="
echo ""
echo "Available launch files:"
echo "  ros2 launch bringup all_systems.launch.py   # Everything"
echo "  ros2 launch bringup control.launch.py       # PID only"
echo "  ros2 launch bringup cv.launch.py            # CV only"
echo "  ros2 launch bringup hardware.launch.py      # Hardware only"
echo ""
echo "Available nodes:"
echo "  ros2 run control depth_pid"
echo "  ros2 run control heading_pid"
echo "  ros2 run control self_arm"
echo "  ros2 run control self_disarm"
echo "  ros2 run cv apriltag_detector_node"
echo "  ros2 run cv fake_camera_node"
echo "  ros2 run cv calibrate_camera"
echo "  ros2 run hardware bluerov2_hardware_interface"
echo "  ros2 run hardware bluerov2_camera_interface"
echo "  ros2 run hardware bluerov2_simulation_interface"
echo "  ros2 run flashing flashing_node"
