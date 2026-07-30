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

## Continental ARS408

已加入 ARS408 Cluster/Object 列表接收节点。它发布 `sensor_msgs/PointCloud2` 到
`/ars408/points_raw` 和 `/ars408/points`，字段包括位置、RCS、相对速度、目标 ID 和
动态属性。坐标严格使用手册定义：雷达前方为 `+x`，雷达接口侧为 `+y`。节点解码
Cluster 状态/通用帧（`0x600`、`0x701`）以及 Object 状态/通用帧（`0x60A`、
`0x60B`）；CAN ID 会按 `sensor_id * 0x10` 自动偏移。

现场 CAN 盒已确认为 USR-CANET200（固件 V1.1.3）。两个 CAN 通道均为 500 kbit/s
正常模式，并配置为 TCP Client，主动连接采集机 `192.168.2.131:29536`。节点默认按现场
设置监听 `29536/TCP`，接收设备实际使用的固定 13 字节 CAN 帧：

```bash
ros2 launch start_collect start.launch.py \
  enable_ars408:=true
```

也可以单独调试：

```bash
ros2 launch radar ars408.launch.py
ros2 topic echo /ars408/status
ros2 topic hz /ars408/points
```

节点同时支持当前现场输出的 Cluster 列表（`0x600`、`0x701`）和 Object 列表
（`0x60A`、`0x60B`）。若网关在本机提供 SocketCAN 接口，则可改用：

```bash
ros2 launch start_collect start.launch.py \
  enable_ars408:=true ars408_source:=socketcan ars408_can_interface:=can0
```

现场雷达当前为 Cluster 输出模式。CAN 总线仍须保证两端各有一个 120 Ω 终端电阻。
若以后切换到 Object 模式并需要正确判断静止/运动目标，手册要求周期输入车辆速度和
偏航率；节点不会臆造这些输入报文。

实时预览和速度过滤：

```bash
ros2 launch radar ars408.launch.py \
  enable_rviz:=true \
  min_speed_mps:=0.0 \
  max_speed_mps:=-1.0 \
  speed_filter_mode:=magnitude
```

ARS408 相对速度的分辨率为 `0.25 m/s`。只查看运动点时，可将
`min_speed_mps` 改为 `0.25`；静止场景设置为 `1.0` 可能没有任何点。

启动文件会发布默认单位静态变换 `ars408_world -> ars408`，RViz 的 Fixed Frame 为
`ars408_world`。这里的单位变换只用于独立预览，不代表已经完成雷达与车辆或激光雷达
之间的外参标定。已知真实安装外参时可传入：

```bash
ros2 launch radar ars408.launch.py enable_rviz:=true \
  parent_frame:=base_link \
  radar_x:=0.0 radar_y:=0.0 radar_z:=0.0 \
  radar_roll:=0.0 radar_pitch:=0.0 radar_yaw:=0.0
```

`/ars408/points_raw` 保留全部点，`/ars408/points` 是过滤后的点。两者的
`PointCloud2` 字段均为 `x`、`y`、`z`、`intensity`（RCS）、`vx`、`vy`、`speed`、
`target_id`、`dynamic_property`。过滤模式可选 `magnitude`（合速度）、`longitudinal`
（纵向速度绝对值）和 `lateral`（横向速度绝对值）；`max_speed_mps:=-1` 表示不设上限。
参数可在运行中修改：

```bash
ros2 param set /ars408_radar min_speed_mps 2.0
ros2 param set /ars408_radar max_speed_mps 20.0
ros2 param set /ars408_radar speed_filter_mode longitudinal
```

保存雷达原始点云、过滤点云、状态和 TF：

```bash
ros2 launch radar ars408.launch.py \
  enable_rviz:=true \
  record_bag:=true
```

默认保存到当前目录的 `bags/ars408_YYYYMMDD_HHMMSS/`，格式为带 Zstd 压缩的 MCAP。
录制器会检查 raw、过滤后点云和状态话题是否都只有一个发布者；若检测到残留的
bag player、速度过滤节点或重复雷达节点，会拒绝录制，避免静默生成混合数据。
也可以指定绝对路径：

```bash
ros2 launch radar ars408.launch.py \
  record_bag:=true \
  bag_output:=/data/ars408_test_001
```

通过总采集启动文件保存雷达时：

```bash
ros2 launch start_collect start.launch.py \
  enable_ars408:=true \
  ars408_record_bag:=true \
  ars408_bag_output:=/data/ars408_test_001
```

结束时使用 `Ctrl+C`，等待 recorder 输出完成后再关闭终端或断电。检查和回放：

```bash
ros2 bag info bags/ars408_YYYYMMDD_HHMMSS
ros2 bag play bags/ars408_YYYYMMDD_HHMMSS
```

回放原始点云并重新设置速度过滤（不要同时播放 bag 内已经过滤好的
`/ars408/points`）。启动文件会把 bag 内的原始点云重映射到隔离话题
`/ars408/playback/points_raw`，避免回放数据混入实时采集：

```bash
ros2 launch radar ars408_playback.launch.py \
  bag_path:=/data/ars408_test_001 \
  enable_rviz:=true \
  loop:=false \
  min_speed_mps:=1.0 \
  max_speed_mps:=30.0 \
  speed_filter_mode:=magnitude
```

播放中可动态修改：

```bash
ros2 param set /ars408_speed_filter min_speed_mps 2.0
ros2 param set /ars408_speed_filter max_speed_mps 20.0
ros2 param set /ars408_speed_filter speed_filter_mode longitudinal
ros2 topic echo /ars408/filter_status
```

## Bag 导出

`bag2pcd.py` 同时支持 ROS 1 `.bag`、ROS 2 SQLite3 和 MCAP：

```bash
source .venv/bin/activate
python ros2imgAndpcd/bag2pcd.py BAG_PATH --output output \
  --image-topic /compressedimg1 --image-topic /compressedimg2 \
  --pointcloud-topic /pointcloud
```
