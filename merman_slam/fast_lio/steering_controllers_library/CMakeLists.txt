cmake_minimum_required(VERSION 3.16)
project(steering_controllers_library LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic -Wconversion)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcpputils REQUIRED)

add_library(
  steering_controllers_library
  SHARED
  src/steering_odometry.cpp
)
target_compile_features(steering_controllers_library PUBLIC cxx_std_17)
target_include_directories(steering_controllers_library PUBLIC
  "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")

ament_target_dependencies(steering_controllers_library
  rclcpp
  rcpputils
)

target_compile_definitions(steering_controllers_library PRIVATE "_USE_MATH_DEFINES")

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS steering_controllers_library
  EXPORT export_evb_steering_controllers_library
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

ament_export_targets(export_evb_steering_controllers_library HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp rcpputils)
ament_package()
