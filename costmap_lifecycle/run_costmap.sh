#!/bin/bash

set -e

source /opt/ros/humble/setup.bash

existing_costmaps="$(ros2 node list 2>/dev/null | grep -Fx '/costmap/costmap' || true)"
if [ -n "$existing_costmaps" ]; then
    echo "ERROR: /costmap/costmap is already running."
    echo "Stop the old costmap process before starting this script."
    exit 1
fi

cleanup() {
    echo ""
    echo "Stopping..."
    kill "$COSTMAP_PID" 2>/dev/null || true
    kill "$TF1_PID" 2>/dev/null || true
    kill "$TF2_PID" 2>/dev/null || true
    wait "$COSTMAP_PID" 2>/dev/null || true
    wait "$TF1_PID" 2>/dev/null || true
    wait "$TF2_PID" 2>/dev/null || true
}

trap cleanup EXIT SIGINT SIGTERM

echo "Starting TF: map -> base_link"
ros2 run tf2_ros static_transform_publisher \
    0 0 0 0 0 0 map base_link &
TF1_PID=$!

echo "Starting TF: base_link -> rslidar"
ros2 run tf2_ros static_transform_publisher \
    0 0 0 0 0 0 base_link rslidar &
TF2_PID=$!

sleep 1

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "Starting Nav2 costmap..."
echo "Using config: $SCRIPT_DIR/costmap.yaml"

ros2 run nav2_costmap_2d nav2_costmap_2d \
    --ros-args \
    -r __node:=costmap \
    --params-file "$SCRIPT_DIR/costmap.yaml" &
COSTMAP_PID=$!

for _ in $(seq 1 20); do
    costmap_count="$(ros2 node list 2>/dev/null | grep -Fc '/costmap/costmap' || true)"
    [ "$costmap_count" -eq 1 ] && break
    sleep 0.5
done
if [ "$costmap_count" -ne 1 ]; then
    echo "ERROR: expected exactly one /costmap/costmap node, found $costmap_count."
    exit 1
fi

echo "Driving costmap lifecycle (Rust)..."
echo "The lifecycle driver will keep running and print one line for every PointCloud2 received."
echo "Listening on /rslidar_points"
ros2 run costmap_lifecycle lifecycle_driver

echo "Running:"
echo "  TF1:     $TF1_PID"
echo "  TF2:     $TF2_PID"
echo "  Costmap: $COSTMAP_PID"
echo ""
echo "Press Ctrl+C to stop."

wait "$COSTMAP_PID"