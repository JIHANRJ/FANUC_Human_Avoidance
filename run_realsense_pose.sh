#!/bin/bash
# Test RealSense Pose Publisher

set -e

cd /home/fanuc/ws_fanuc

# Source ROS 2
source /opt/ros/jazzy/setup.bash

echo ""
echo "====== RealSense Pose Matrix Publisher ======"
echo ""
echo "This will:"
echo "1. Start RealSense depth + color streaming"
echo "2. Run MediaPipe pose detection"
echo "3. Publish pose matrix to /realsense/pose_matrix topic"
echo ""
echo "In another terminal, you can listen with:"
echo "  ros2 topic echo /realsense/pose_matrix"
echo ""
read -p "Press ENTER to start..."
echo ""

python3 /home/fanuc/ws_fanuc/realsense_pose_node.py
