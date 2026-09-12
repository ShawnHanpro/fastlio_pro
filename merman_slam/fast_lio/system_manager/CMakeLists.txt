cmake_minimum_required(VERSION 3.8)
project(system_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(scancontext_ros2 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters kdtree)

add_executable(mode_manager
  src/mode_manager.cpp
)
ament_target_dependencies(mode_manager
  rclcpp
  std_srvs
)

add_executable(scan_context_relocalization
  src/scan_context_relocalization.cpp
)

target_include_directories(scan_context_relocalization PRIVATE
  ${PCL_INCLUDE_DIRS}
)

target_link_libraries(scan_context_relocalization
  ${PCL_LIBRARIES}
)

ament_target_dependencies(scan_context_relocalization
  rclcpp
  std_srvs
  sensor_msgs
  geometry_msgs
  livox_ros_driver2
  pcl_conversions
  tf2
  tf2_ros
  scancontext_ros2
)

install(TARGETS
  mode_manager
  scan_context_relocalization
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
