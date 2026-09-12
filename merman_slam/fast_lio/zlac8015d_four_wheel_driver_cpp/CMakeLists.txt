cmake_minimum_required(VERSION 3.8)
project(zlac8015d_four_wheel_driver_cpp)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/FourWheelState.msg"
  "srv/SetFourWheelSpeeds.srv"
  DEPENDENCIES std_msgs
)

add_executable(four_wheel_driver_node src/four_wheel_driver_node.cpp)
ament_target_dependencies(four_wheel_driver_node rclcpp std_msgs std_srvs)
rosidl_target_interfaces(four_wheel_driver_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS four_wheel_driver_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(rosidl_default_runtime)
ament_package()
