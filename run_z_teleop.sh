#!/bin/bash
# Simple launcher for Z-axis keyboard teleop with CRX-10iA/L in simulation

set -e

cd /home/fanuc/ws_fanuc

# Source ROS 2 environment
source /opt/ros/jazzy/setup.bash
source install/setup.bash

echo ""
echo "====== Z-Axis Keyboard Teleop (Simulation) ======"
echo "Robot: CRX-10iA/L"
echo "Mode: MoveIt Simulation (RViz, no real hardware)"
echo ""
echo "This launcher will:"
echo "1. Start fanuc_moveit_config with mock hardware"
echo "2. Load MoveIt with CRX-10iA/L"
echo "3. Start RViz visualization"
echo "4. Start keyboard teleop in a separate terminal"
echo ""
echo "Steps:"
echo "Step 1: Close the first terminal when MoveIt is loaded (you'll see RViz window)"
echo "Step 2: Use UP/DOWN arrows in the terminal for Z control"
echo "Step 3: Watch RViz for the visualization"
echo ""
read -p "Press ENTER to continue..."
echo ""

# Launch MoveIt with CRX-10iA/L in simulation mode
echo "[1/2] Launching MoveIt + RViz..."
ros2 launch fanuc_moveit_config fanuc_moveit.launch.py \
    robot_model:=crx10ia_l \
    use_mock:=true &

MOVEIT_PID=$!

# Give MoveIt time to start
sleep 5

# Launch keyboard teleop in new terminal
echo "[2/2] Starting keyboard teleop..."
xterm -hold -e "cd /home/fanuc/ws_fanuc && source /opt/ros/jazzy/setup.bash && source install/setup.bash && python3 simple_z_teleop.py" &

TELEOP_PID=$!

echo ""
echo "====== System Running ======"
echo "MoveIt PID: $MOVEIT_PID"
echo "Teleop PID: $TELEOP_PID"
echo ""
echo "To stop: Close the RViz window and press Ctrl+C here"
echo ""

# Wait for processes
wait $MOVEIT_PID $TELEOP_PID 2>/dev/null || true

echo ""
echo "Shutdown complete"
