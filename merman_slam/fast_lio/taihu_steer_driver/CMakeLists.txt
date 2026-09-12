cmake_minimum_required(VERSION 3.10)
project(taihu_steer_driver)

if(POLICY CMP0167)
  cmake_policy(SET CMP0167 NEW)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_default_runtime REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(Boost REQUIRED program_options)

if(NOT TARGET ethercat-cpp::ethercat-cpp)
  find_package(
    ethercat-cpp
    REQUIRED
    PATHS
    "${CMAKE_CURRENT_SOURCE_DIR}/ecat_com"
    NO_DEFAULT_PATH
    NO_CMAKE_FIND_ROOT_PATH
  )
endif()

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DriverState.msg"
  DEPENDENCIES std_msgs
)

include_directories(
  ${PROJECT_SOURCE_DIR}
  /usr/include/eigen3
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/datp/datp
  ${PROJECT_SOURCE_DIR}/ecat_com/include
)

link_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/datp
  ${CMAKE_CURRENT_SOURCE_DIR}/ecat_com/lib
)

file(GLOB DATP_LIBS "${CMAKE_CURRENT_SOURCE_DIR}/datp/libdatp.so")

set(SRC_FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/ecat_master.cpp
)

add_executable(taihu_steer_driver_node ${SRC_FILES})

ament_target_dependencies(taihu_steer_driver_node
  rclcpp
  std_msgs
  std_srvs
  sensor_msgs
  Eigen3
  ament_index_cpp
  rosidl_default_runtime
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

target_link_libraries(taihu_steer_driver_node
  ${DATP_LIBS}
  ethercat-cpp::ethercat-cpp
  spdlog::spdlog
  ${cpp_typesupport_target}
  -pthread
  -lrt
)

install(
  FILES
    ${DATP_LIBS}
    "${CMAKE_CURRENT_SOURCE_DIR}/ecat_com/lib/libethercat-cpp.so"
    "${CMAKE_CURRENT_SOURCE_DIR}/ecat_com/lib/libethercat-cpp.so.4.1.4"
  DESTINATION lib
)

install(
  TARGETS taihu_steer_driver_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
