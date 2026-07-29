cmake_minimum_required(VERSION 3.8)
project(mess_sync)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters)

add_executable(mess_sync src/mess_sync.cpp)
target_compile_features(mess_sync PUBLIC cxx_std_17)
target_include_directories(mess_sync PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(mess_sync ${PCL_LIBRARIES})
ament_target_dependencies(mess_sync rclcpp sensor_msgs pcl_conversions)

install(TARGETS mess_sync DESTINATION lib/${PROJECT_NAME})
ament_package()
