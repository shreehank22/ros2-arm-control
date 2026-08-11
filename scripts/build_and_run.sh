#!/bin/bash
# Usage: ./scripts/build_and_run.sh <package_name> <node_name> [duration_seconds]

set -e

PACKAGE=$1
NODE=$2
DURATION=${3:-5}

if [ -z "$PACKAGE" ] || [ -z "$NODE" ]; then
    echo "Usage: $0 <package_name> <node_name> [duration_seconds]"
    exit 1
fi

echo "=== Building $PACKAGE ==="
colcon build --symlink-install --packages-select "$PACKAGE"

echo "=== Sourcing workspace ==="
source install/setup.bash

echo "=== Running $NODE for ${DURATION}s ==="
ros2 run "$PACKAGE" "$NODE" > /tmp/${NODE}_output.log 2>&1 &
PID=$!
sleep "$DURATION"
kill -INT $PID
sleep 1

echo "=== Output ==="
cat /tmp/${NODE}_output.log
