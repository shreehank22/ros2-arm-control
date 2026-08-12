#!/bin/bash
# Usage: ./scripts/clean_reset.sh
# Kills all lingering nodes from this workspace and wipes build artifacts.

set -e

WS_DIR="$HOME/ros2-arm-control"

echo "=== Killing lingering processes ==="
for proc in sim_node controller_node traj_gen_node rviz2 robot_state_publisher joint_state_publisher_gui rqt_plot; do
    if pgrep -f "$proc" > /dev/null; then
        echo "Killing $proc..."
        pkill -9 -f "$proc"
    else
        echo "$proc: nothing running"
    fi
done

echo ""
echo "=== Verifying nothing left ==="
sleep 1
REMAINING=$(ps aux | grep -E "sim_node|controller_node|traj_gen_node|rviz2|robot_state_publisher|joint_state_publisher_gui|rqt_plot" | grep -v grep || true)
if [ -n "$REMAINING" ]; then
    echo "WARNING: some processes survived:"
    echo "$REMAINING"
else
    echo "Clean — nothing running."
fi

echo ""
echo "=== Wiping build artifacts ==="
cd "$WS_DIR"
rm -rf build install log
echo "Removed build/, install/, log/"

echo ""
echo "=== Unsetting potentially stale environment variables ==="
unset AMENT_PREFIX_PATH
unset COLCON_PREFIX_PATH
unset COLCON_CURRENT_PREFIX
echo "Cleared AMENT_PREFIX_PATH, COLCON_PREFIX_PATH, COLCON_CURRENT_PREFIX for this shell"

echo ""
echo "=== Done. Next steps: ==="
echo "  colcon build --symlink-install"
echo "  source install/setup.zsh   (interactive shell)"
echo "  source install/setup.bash  (inside bash scripts)"