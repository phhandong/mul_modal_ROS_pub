#!/usr/bin/env bash
set -eo pipefail

bag_name="${1:-00}"
data_root="/home/handong/Code/406mul_camera_ROS_123/data_0729/cam_radar_ros2_rviz"
bag_path="${data_root}/${bag_name}"
rviz_config="${data_root}/cam_radar.rviz"

if [[ ! -f "${bag_path}/metadata.yaml" ]]; then
  echo "Bag not found: ${bag_path}" >&2
  echo "Available bags: 00 01 02 03 04 05" >&2
  exit 2
fi

source /opt/ros/jazzy/setup.bash
set -u

ros2 run image_transport republish \
  --ros-args \
  -p in_transport:=compressed \
  -p out_transport:=raw \
  --remap in/compressed:=/camera/image/compressed \
  --remap out:=/camera/image &
republisher_pid=$!

rviz2 -d "${rviz_config}" &
rviz_pid=$!
trap 'kill "${rviz_pid}" "${republisher_pid}" 2>/dev/null || true' EXIT INT TERM

sleep 2
ros2 bag play "${bag_path}" --loop
