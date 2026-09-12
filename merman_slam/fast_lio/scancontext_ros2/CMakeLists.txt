cmake_minimum_required(VERSION 3.5)
project(scancontext_ros2)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/scancontext_ros2/Scancontext.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${PCL_INCLUDE_DIRS}
)

target_compile_definitions(${PROJECT_NAME} PRIVATE ${PCL_DEFINITIONS})

target_link_libraries(${PROJECT_NAME}
  ${PCL_LIBRARIES}
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  pcl_conversions
)

install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(rclcpp pcl_conversions)

ament_package()
