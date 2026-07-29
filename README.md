# 多传感器采集（ROS 2 Jazzy）

本工作区面向 Ubuntu 24.04 / ROS 2 Jazzy，采集 RoboSense RS128、海康单目可见光相机、海康双光谱云台相机，以及可选的 Quantum、IMU、OpenCPN NMEA/AIS 数据。

## 依赖

先安装 ROS 2 官方驱动与构建工具：

```bash
sudo apt update
sudo apt install -y python3-colcon-common-extensions python3-rosdep \
  ros-jazzy-rslidar-sdk ros-jazzy-rslidar-msg ros-jazzy-nmea-navsat-driver \
  ros-jazzy-pcl-ros ros-jazzy-pcl-conversions ros-jazzy-cv-bridge \
  ros-jazzy-image-transport-plugins
```

离线工具使用项目虚拟环境：

```bash
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
python -m pip install -r ros2imgAndpcd/requirements.txt
```

## 构建

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

`src/rslidar_sdk` 与 `src/nmea_navsat_driver` 是保留的 ROS 1 历史副本，已用 `COLCON_IGNORE` 排除；运行时使用 Jazzy 官方包。

## 海康相机

复制并填写本地配置，配置文件不会被 Git 跟踪：

```bash
cp src/start_collect/config/hik_mono.yaml src/start_collect/config/hik_mono.local.yaml
cp src/start_collect/config/hik_bispectral.yaml src/start_collect/config/hik_bispectral.local.yaml
cp src/start_collect/config/rslidar_rs128.yaml src/start_collect/config/rslidar_rs128.local.yaml
```

现场配置已写入本机忽略文件：单目 `.110`、双光谱 `.113`、RS128 `.114`。本机配置可设置 `password`；该字段优先于 `password_env`。模板和 Git 仍不保存密码。

```bash
ros2 launch start_collect start.launch.py \
  rslidar_config:=$PWD/src/start_collect/config/rslidar_rs128.local.yaml \
  mono_config:=$PWD/src/start_collect/config/hik_mono.local.yaml \
  bispectral_config:=$PWD/src/start_collect/config/hik_bispectral.local.yaml
```

默认话题：

- `/hik_mono/image/compressed`
- `/hik_bispectral/visible/image/compressed`
- `/hik_bispectral/thermal/image/compressed`
- `/hik_bispectral/temperature/image_raw`（`32FC1`）
- `/p2p_data`、`/ptz_ctrl`
- `/rslidar_points`、`/pointcloud`

所有硬件均可通过 `enable_lidar:=false`、`enable_mono:=false`、`enable_bispectral:=false` 等 launch 参数单独关闭。

## Bag 导出

`bag2pcd.py` 同时支持 ROS 1 `.bag`、ROS 2 SQLite3 和 MCAP：

```bash
source .venv/bin/activate
python ros2imgAndpcd/bag2pcd.py BAG_PATH --output output \
  --image-topic /compressedimg1 --image-topic /compressedimg2 \
  --pointcloud-topic /pointcloud
```
