#!/usr/bin/env bash
# Start only the field RS128 lidar and Hikvision mono camera.
set -eo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
  echo "ROS 2 Jazzy is not installed at /opt/ros/jazzy." >&2
  exit 1
fi
if [[ ! -f "${workspace_dir}/install/setup.bash" ]]; then
  echo "Workspace is not built. Run: cd ${workspace_dir} && colcon build --symlink-install" >&2
  exit 1
fi

lidar_config="${workspace_dir}/src/start_collect/config/rslidar_rs128.local.yaml"
mono_config="${workspace_dir}/src/start_collect/config/hik_mono.local.yaml"
for config_file in "${lidar_config}" "${mono_config}"; do
  if [[ ! -f "${config_file}" ]]; then
    echo "Required local configuration is missing: ${config_file}" >&2
    exit 1
  fi
done

source /opt/ros/jazzy/setup.bash
source "${workspace_dir}/install/setup.bash"
set -u

exec ros2 launch start_collect start.launch.py \
  enable_lidar:=true \
  enable_mono:=true \
  enable_bispectral:=false \
  enable_sync:=false \
  enable_radar:=false \
  enable_imu:=false \
  enable_nmea:=false \
  enable_rviz:=false \
  rslidar_config:="${lidar_config}" \
  mono_config:="${mono_config}"
