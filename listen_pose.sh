#!/bin/bash
# Listen to RealSense Pose Matrix topic

source /opt/ros/jazzy/setup.bash

echo "Listening to /realsense/pose_matrix topic..."
echo "Press Ctrl+C to stop"
echo ""

ros2 topic echo /realsense/pose_matrix
