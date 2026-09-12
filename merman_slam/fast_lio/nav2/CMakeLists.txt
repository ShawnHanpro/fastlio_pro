cmake_minimum_required(VERSION 3.8)
project(nav2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# Install directories
install(DIRECTORY
  launch
  param
  rviz
  DESTINATION share/${PROJECT_NAME}/
)

# map 目录已删除（包含1.7GB的数据库文件）
# install(DIRECTORY
#   map
#   DESTINATION share/${PROJECT_NAME}/
# )

# Install scripts (Swerve mode only)
install(PROGRAMS
  scripts/gazebo_gt_odom_publisher.py
  scripts/swerve_odom_publisher.py
  scripts/scan_motion_compensation.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

