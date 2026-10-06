cmake_minimum_required(VERSION 3.8)
project(decision_ftg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pure_pursuit REQUIRED)

add_executable(decision_ftg_node src/decision_ftg_node.cpp)
ament_target_dependencies(decision_ftg_node
  rclcpp
  ackermann_msgs
  geometry_msgs
  nav_msgs
  pure_pursuit
)

target_include_directories(decision_ftg_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS
  decision_ftg_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

ament_package()
