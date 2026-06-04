#!/bin/bash
# revo_scoutlab_teleop.sh
# Launches agv_pro_bringup in the background, then runs LAB/teleop.py
# in the foreground. Ctrl-C (or any exit) cleans up bringup.

set -e

LAB_DIR="${LAB_DIR:-$HOME/Revobots/aditya/aadi_scout/LAB}"
BRINGUP_PID=""

cleanup() {
    # Disable the trap so we don't recurse if cleanup itself errors
    trap - SIGINT SIGTERM EXIT

    if [ -n "$BRINGUP_PID" ] && kill -0 "$BRINGUP_PID" 2>/dev/null; then
        echo
        echo "[teleop_start] Stopping agv_pro_bringup (PID: $BRINGUP_PID)..."
        kill -TERM "$BRINGUP_PID" 2>/dev/null || true

        # Give it 5s to exit gracefully, then force-kill
        for _ in 1 2 3 4 5; do
            kill -0 "$BRINGUP_PID" 2>/dev/null || break
            sleep 1
        done
        kill -KILL "$BRINGUP_PID" 2>/dev/null || true
        wait "$BRINGUP_PID" 2>/dev/null || true
    fi

    echo "[teleop_start] Cleanup complete."
}

trap cleanup SIGINT SIGTERM EXIT

# ── 1. Source ROS 2 ──────────────────────────────────────────────────────────
echo "[teleop_start] Sourcing ROS 2 Humble and local workspace..."
source /opt/ros/humble/setup.bash
source "$HOME/agv_pro_ros2/install/local_setup.bash"

# ── 2. Launch agv_pro_bringup in the background ──────────────────────────────
echo "[teleop_start] Launching agv_pro_bringup..."
ros2 launch agv_pro_bringup agv_pro_bringup.launch.py &
BRINGUP_PID=$!

# Give bringup time to bring up /cmd_vel, TF, etc.
echo "[teleop_start] Waiting 5s for ROS nodes to initialize..."
sleep 5

# Fail fast if bringup died during init
if ! kill -0 "$BRINGUP_PID" 2>/dev/null; then
    echo "[teleop_start] ERROR: agv_pro_bringup exited during startup."
    exit 1
fi

# ── 3. Run teleop in the foreground ──────────────────────────────────────────
echo "[teleop_start] Starting LAB/teleop.py..."
cd "$LAB_DIR"
exec python3 teleop.py