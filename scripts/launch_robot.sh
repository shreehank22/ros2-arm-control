#!/bin/zsh
# Usage: ./launch_robot.sh
# Starts sim_node, robot_state_publisher+rviz, and foxglove_bridge, then opens
# Foxglove in the browser. Waits for you to confirm before starting the
# controller, so you can see the arm in Foxglove before control kicks in.

WS_DIR="$HOME/ros2-arm-control"
LOG_DIR="/tmp/ros2_arm_logs"
mkdir -p "$LOG_DIR"

PIDS=()

cleanup() {
    echo ""
    echo "=== Shutting down all nodes ==="
    for pid in "${PIDS[@]}"; do
        kill -9 "$pid" 2>/dev/null
    done
    pkill -9 -f sim_node 2>/dev/null
    pkill -9 -f controller_node 2>/dev/null
    pkill -9 -f rviz2 2>/dev/null
    pkill -9 -f robot_state_publisher 2>/dev/null
    pkill -9 -f foxglove_bridge 2>/dev/null
    echo "Done."
}
trap cleanup EXIT INT TERM

mamba activate ros2_jazzy
cd "$WS_DIR"
source install/setup.zsh

echo "=== Starting sim_node ==="
ros2 run arm_sim sim_node > "$LOG_DIR/sim_node.log" 2>&1 &
PIDS+=($!)
sleep 1

echo "=== Starting robot_state_publisher + rviz (rviz window may not render, ignore it) ==="
LIBGL_ALWAYS_SOFTWARE=1 ros2 launch arm_description display.launch.py > "$LOG_DIR/display.log" 2>&1 &
PIDS+=($!)
sleep 2

echo "=== Starting foxglove_bridge ==="
ros2 launch foxglove_bridge foxglove_bridge_launch.xml > "$LOG_DIR/foxglove_bridge.log" 2>&1 &
PIDS+=($!)

echo "=== Waiting for foxglove_bridge to be ready ==="
sleep 3

FOXGLOVE_URL="https://studio.foxglove.dev/connect?ds=foxglove-websocket&ds.url=ws://localhost:8765"
echo "=== Opening Foxglove: $FOXGLOVE_URL ==="
xdg-open "$FOXGLOVE_URL" 2>/dev/null &

echo ""
echo "=== sim_node, robot_state_publisher, and foxglove_bridge are running. ==="
echo "=== Go check Foxglove now (enable /robot_description if needed). ==="
echo ""
read "?Press ENTER once you're ready to start controller_node..."

echo "=== Starting controller_node ==="
ros2 run arm_control controller_node > "$LOG_DIR/controller_node.log" 2>&1 &
PIDS+=($!)

echo ""
echo "=== controller_node is now running. Watch the arm settle in Foxglove. ==="
echo "=== Press Ctrl+C to stop everything ==="

wait