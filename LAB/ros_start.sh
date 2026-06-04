#!/bin/bash

# Exit immediately if a command exits with a non-zero status
set -e

# Define a cleanup function to elegantly stop the background process
cleanup() {
    echo -e "\n[ros_start] Stopping agv_pro_bringup (PID: $BRINGUP_PID)..."
    kill -TERM "$BRINGUP_PID" 2>/dev/null || true
    wait "$BRINGUP_PID" 2>/dev/null || true
    echo "[ros_start] Cleanup complete. Exiting."
    exit 0
}

# Trap SIGINT (Ctrl+C) and SIGTERM so the cleanup function runs when the script exits
trap cleanup SIGINT SIGTERM EXIT

# 1. Source ROS2 workspace
echo "[ros_start] Sourcing ROS 2 Humble and local workspace..."
source /opt/ros/humble/setup.bash
source ~/agv_pro_ros2/install/local_setup.bash

# 2. Launch agv_pro_bringup in the background
echo "[ros_start] Launching agv_pro_bringup..."
ros2 launch agv_pro_bringup agv_pro_bringup.launch.py &
BRINGUP_PID=$!

# Give bringup time to start its nodes
echo "[ros_start] Waiting 5 seconds for nodes to initialize..."
sleep 5

# ---------------------------------------------------------
# 3. RUN YOUR MOTION SCRIPT HERE
# ---------------------------------------------------------
# Replace the command below with your actual motion script.
# Because of the 'trap' above, when this command finishes 
# (or if it crashes), the trap will automatically kill the 
# bringup process before exiting.

echo "[ros_start] Starting motion script..."

# Example placeholder (Uncomment and replace with your actual command):
# ros2 run my_motion_pkg motion_node
# OR
# python3 ~/my_scripts/motion_script.py

# If you don't have a motion script yet and just want to keep 
# the bringup running until you press Ctrl+C, use the line below:
wait "$BRINGUP_PID"
