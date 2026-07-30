#!/usr/bin/env bash
set -eo pipefail

bag_name="${1:-0}"
calib_root="/home/handong/Code/406mul_camera_ROS_123/data_0729/cam_lidar_ros2_calib"
data_root="${calib_root}/normalized"
player_root="${calib_root}/rviz_player"
bag_path="${data_root}/${bag_name}"

if [[ ! -f "${bag_path}/metadata.yaml" ]]; then
  echo "Bag not found: ${bag_path}" >&2
  echo "Available bags: 0 1 2 3 4 5 6 7 8" >&2
  exit 2
fi

source /opt/ros/jazzy/setup.bash
set -u

ros2 run image_transport republish \
  --ros-args \
  -p in_transport:=compressed \
  -p out_transport:=raw \
  --remap in/compressed:=/sync/image_visible/compressed \
  --remap out:=/camera/image &
republisher_pid=$!

rviz2 -d "${player_root}/cam_lidar.rviz" &
rviz_pid=$!
trap 'kill "${rviz_pid}" "${republisher_pid}" 2>/dev/null || true' EXIT INT TERM

sleep 2
ros2 bag play "${bag_path}" --loop
