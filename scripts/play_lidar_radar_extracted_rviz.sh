#!/usr/bin/env bash
set -eo pipefail

sequence="${1:-10}"
data_root="/home/handong/Code/406mul_camera_ROS_123/data_0729/lidar_radar_extracted"
player_root="${data_root}/rviz_player"
sequence_dir="${data_root}/${sequence}"

if [[ ! -d "${sequence_dir}/lidar" || ! -d "${sequence_dir}/radar" ]]; then
  echo "Sequence not found: ${sequence_dir}" >&2
  echo "Available sequences: 10 11 12 13 14 20 21" >&2
  exit 2
fi

source /opt/ros/jazzy/setup.bash
set -u

rviz2 -d "${player_root}/lidar_radar_extracted.rviz" &
rviz_pid=$!

sleep 2
/usr/bin/python3 "${player_root}/play_extracted_lidar_radar.py" \
  "${sequence_dir}" \
  --loop \
  --hz 10 \
  --max-lidar-points 80000 &
player_pid=$!

trap 'kill "${rviz_pid}" "${player_pid}" 2>/dev/null || true' EXIT INT TERM
wait -n "${rviz_pid}" "${player_pid}"
