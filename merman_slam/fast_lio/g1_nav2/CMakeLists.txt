cmake_minimum_required(VERSION 3.16)
project(g1_nav2 LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(
  pseudo_diff_4wis_controller
  src/pseudo_diff_4wis_controller.cpp
)

ament_target_dependencies(
  pseudo_diff_4wis_controller
  rclcpp
  geometry_msgs
  sensor_msgs
  std_msgs
)

install(
  TARGETS pseudo_diff_4wis_controller
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config behavior_trees
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
