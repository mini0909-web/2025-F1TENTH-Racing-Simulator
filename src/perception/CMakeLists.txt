cmake_minimum_required(VERSION 3.8)
project(perception_dbscan)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pure_pursuit REQUIRED)

add_executable(perception_dbscan_node src/perception_dbscan_node.cpp)
ament_target_dependencies(perception_dbscan_node
  rclcpp
  sensor_msgs
  geometry_msgs
  pure_pursuit
)

install(TARGETS
  perception_dbscan_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
