cmake_minimum_required(VERSION 3.16)
project(ackermann_steering_controller LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic -Wconversion)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(steering_controllers_library REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(zlac8015d_four_wheel_driver_cpp REQUIRED)

set(ALL_DEPS
  rclcpp
  std_msgs
  steering_controllers_library
  nav_msgs
  sensor_msgs
  tf2
  tf2_ros
  geometry_msgs
  zlac8015d_four_wheel_driver_cpp
)

add_executable(ackermann_steering_controller_node src/ackermann_kinematics_node.cpp)
ament_target_dependencies(ackermann_steering_controller_node
  ${ALL_DEPS}
)

install(
  TARGETS ackermann_steering_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
