cmake_minimum_required(VERSION 3.8)
project(pure_pursuit)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(racecar_simulator REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files
  msg/Obstacle.msg
  msg/ObstacleArray.msg
  msg/ObstacleCluster.msg
  msg/ObstacleDecision.msg
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES std_msgs geometry_msgs
)

add_executable(pure_pursuit_node src/pure_pursuit.cpp)
add_executable(racing_line_publisher src/racing_line_publisher.cpp)

ament_target_dependencies(pure_pursuit_node
  rclcpp
  ackermann_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  tf2
  racecar_simulator
)

ament_target_dependencies(racing_line_publisher
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
  tf2
  racecar_simulator
  ament_index_cpp
)

rosidl_target_interfaces(pure_pursuit_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS
  pure_pursuit_node
  racing_line_publisher
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()