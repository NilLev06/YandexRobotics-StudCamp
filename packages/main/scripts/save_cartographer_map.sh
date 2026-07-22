#!/usr/bin/env bash
set -euo pipefail

OUTPUT_STEM="${1:-/src/maps/rover_map}"
RESOLUTION="${2:-0.025}"
PBSTREAM="${OUTPUT_STEM}.pbstream"

mkdir -p "$(dirname "$OUTPUT_STEM")"

echo "[1/3] Saving the current ROS OccupancyGrid..."
ros2 run nav2_map_server map_saver_cli \
  -f "$OUTPUT_STEM" \
  --ros-args -p map_subscribe_transient_local:=true

echo "[2/3] Serializing Cartographer state to $PBSTREAM..."
ros2 service call /write_state cartographer_ros_msgs/srv/WriteState \
  "{filename: '$PBSTREAM', include_unfinished_submaps: true}"

echo "[3/3] Verifying output files..."
for file in "${OUTPUT_STEM}.yaml" "${OUTPUT_STEM}.pgm" "$PBSTREAM"; do
  test -s "$file" || { echo "Missing or empty: $file" >&2; exit 1; }
done

echo "Saved:"
printf '  %s\n' "${OUTPUT_STEM}.yaml" "${OUTPUT_STEM}.pgm" "$PBSTREAM"
echo "Resolution used by live map: ${RESOLUTION} m/cell"
