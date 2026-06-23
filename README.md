# FANUC Human Avoidance

ROS 2 Jazzy workspace for visualizing a MediaPipe human skeleton from an Intel RealSense D456 in RViz alongside a FANUC CRX-10iA/L robot.

The current human-tracking path publishes RViz `visualization_msgs/MarkerArray` messages on `/human_markers`. Landmarks are red spheres and MediaPipe pose connections are white `LINE_LIST` segments in the `camera_link` frame.

## Workspace Contents

- `src/human_tracker`: RealSense + MediaPipe pose extraction and RViz marker publishing.
- `src/hand_teleop_z`: Z-axis hand teleoperation experiments.
- `src/fanuc_driver`: FANUC ROS 2 driver, tracked as an upstream submodule.
- `src/fanuc_description`: FANUC robot descriptions, tracked as an upstream submodule.
- `realsense_pose_node.py`, `z_axis_extractor.py`, `simple_z_teleop.py`: standalone development scripts.

## Requirements

- Ubuntu with ROS 2 Jazzy
- FANUC driver dependencies
- Intel RealSense D456 and librealsense/pyrealsense2
- Python virtual environment with:
  - `mediapipe==0.10.21`
  - `opencv-contrib-python`
  - `numpy`
  - `pyrealsense2`

MediaPipe `0.10.21` is used because the tracker currently depends on the classic `mp.solutions.pose` API.

## Clone

```bash
git clone --recurse-submodules https://github.com/JIHANRJ/FANUC_Human_Avoidance.git
cd FANUC_Human_Avoidance
```

If the repository was cloned without submodules:

```bash
git submodule update --init --recursive
```

## Build

```bash
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

## Run Human Skeleton Markers

Start the static transform used by RViz:

```bash
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 world camera_link
```

Run the pose publisher:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source .venv/bin/activate

python -m human_tracker.human_pose_publisher
```

For direct script execution during development:

```bash
cd src/human_tracker/human_tracker
python human_pose_publisher.py
```

## RViz Setup

- Fixed Frame: `world`
- Robot display: normal FANUC/MoveIt configuration
- Add display: `MarkerArray`
- Topic: `/human_markers`

Expected marker output:

- Red dots: valid 3D MediaPipe landmarks
- White lines: valid MediaPipe pose connections
- Frame: `camera_link`

Useful checks:

```bash
ros2 node list
ros2 topic hz /human_markers
ros2 topic echo /human_markers --once
ros2 run tf2_ros tf2_echo world camera_link
```

## Notes

If only a few skeleton points or links appear, RealSense depth is likely invalid for some MediaPipe landmarks. Invalid landmarks are deleted from RViz instead of publishing NaN marker positions.

If the skeleton appears near the robot origin, verify the physical `world -> camera_link` transform. The identity transform is useful for testing but does not represent the real camera pose unless the camera is actually at the world origin.
