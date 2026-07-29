cmake_minimum_required(VERSION 3.8)
project(message_interface)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AIS.msg"
  "msg/EnvData.msg"
  "msg/Ownship.msg"
  "msg/P2PData.msg"
  "msg/PtzCtrl.msg"
  "msg/RadarTarget.msg"
  "msg/Spoke.msg"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
