#!/usr/bin/env bash
set -eo pipefail

exec 9>/tmp/manual_lidar_radar_translation.lock
if ! flock -n 9; then
  echo "Another manual calibration session is already running." >&2
  echo "Close it with ESC before selecting a different frame." >&2
  exit 2
fi

sequence="${1:-10}"
frame="${2:-0}"
data_root="/home/handong/Code/406mul_camera_ROS_123/data_0729/lidar_radar_extracted"
tool_root="${data_root}/manual_translation_calib"
sequence_dir="${data_root}/${sequence}"

if [[ ! -d "${sequence_dir}/lidar" || ! -d "${sequence_dir}/radar" ]]; then
  echo "Sequence not found: ${sequence_dir}" >&2
  echo "Available sequences: 10 11 12 13 14 20 21" >&2
  exit 2
fi

source /opt/ros/jazzy/setup.bash
set -u

rviz2 -d "${tool_root}/manual_lidar_radar_translation.rviz" &
rviz_pid=$!
trap 'kill "${rviz_pid}" 2>/dev/null || true' EXIT INT TERM

sleep 2
load_args=()
if [[ -f "${tool_root}/results/CURRENT_TRANSLATION.yaml" ]]; then
  load_args=(--load "${tool_root}/results/CURRENT_TRANSLATION.yaml")
fi
/usr/bin/python3 "${tool_root}/manual_lidar_radar_translation.py" \
  "${sequence_dir}" \
  --frame "${frame}" \
  --output-dir "${tool_root}/results" \
  --step 0.1 \
  --max-lidar-points 120000 \
  "${load_args[@]}"
