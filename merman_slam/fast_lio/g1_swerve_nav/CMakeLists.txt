cmake_minimum_required(VERSION 3.8)
project(g1_swerve_nav)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(zlac8015d_four_wheel_driver_cpp REQUIRED)

add_library(swerve_kinematics src/swerve_kinematics.cpp)
target_include_directories(swerve_kinematics PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(swerve_kinematics PUBLIC cxx_std_17)

add_executable(swerve_controller src/swerve_controller_node.cpp)
target_link_libraries(swerve_controller swerve_kinematics)
target_compile_features(swerve_controller PUBLIC cxx_std_17)
ament_target_dependencies(swerve_controller
  rclcpp geometry_msgs sensor_msgs std_msgs std_srvs
  zlac8015d_four_wheel_driver_cpp)

add_executable(cmd_vel_arbiter src/cmd_vel_arbiter.cpp)
target_compile_features(cmd_vel_arbiter PUBLIC cxx_std_17)
ament_target_dependencies(cmd_vel_arbiter rclcpp geometry_msgs std_srvs)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS
  swerve_kinematics
  swerve_controller
  cmd_vel_arbiter
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch behavior_trees DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/verify_runtime.sh DESTINATION lib/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_package()
