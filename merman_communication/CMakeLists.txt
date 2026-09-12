cmake_minimum_required(VERSION 3.22)

project(merman_communication LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# =========================
# common 路径
# =========================
set(MERMAN_COMMON
    ${CMAKE_CURRENT_SOURCE_DIR}/../merman_common
)

# 让 CMake 能找到：
# ${MERMAN_COMMON}/share/merman_ros2_types/cmake/merman_ros2_typesConfig.cmake
list(PREPEND CMAKE_PREFIX_PATH "${MERMAN_COMMON}")
find_package(merman_ros2_types REQUIRED)


# =========================
# ROS2 依赖
# 注意：
# 这不会把你的工程变成 ROS2 package
# 只是使用 ROS2 的头文件、库和 CMake 配置
# 编译前需要 source /opt/ros/$ROS_DISTRO/setup.bash
# =========================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)

# 如果后面用到了其他 ROS2 msg，再继续加：
# find_package(geometry_msgs REQUIRED)
# find_package(nav_msgs REQUIRED)
# find_package(sensor_msgs REQUIRED)

# =========================
# 源文件
# =========================
file(GLOB_RECURSE COMMUNICATION_SRC
    CONFIGURE_DEPENDS
    src/*.cpp
)

# =========================
# 生成动态库
# =========================
add_library(
    merman_communication
    SHARED
    ${COMMUNICATION_SRC}
)

# =========================
# 输出路径 + RPATH
# =========================
set_target_properties(merman_communication PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${MERMAN_COMMON}/lib"
    ARCHIVE_OUTPUT_DIRECTORY "${MERMAN_COMMON}/lib"
    RUNTIME_OUTPUT_DIRECTORY "${MERMAN_COMMON}/bin"

    BUILD_RPATH "\$ORIGIN"
    INSTALL_RPATH "\$ORIGIN"
    BUILD_WITH_INSTALL_RPATH TRUE
)

# =========================
# 头文件路径
# =========================
target_include_directories(
    merman_communication
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/include/merman_communication_interface

    PRIVATE
        ${MERMAN_COMMON}/include
        ${MERMAN_COMMON}/third_party/nlohmann_json/include
        ${MERMAN_COMMON}/types
        ${MERMAN_COMMON}/third_party/arm64/include/eigen3
)

# =========================
# ROS2 依赖链接
# =========================
ament_target_dependencies(
    merman_communication
    rclcpp
    std_msgs
    std_srvs
    geometry_msgs
    rclcpp_action
    nav2_msgs
    merman_ros2_types
)

# =========================
# 注入模块名供 MermanLogger 自动识别
# =========================
target_compile_definitions(
    merman_communication
    PRIVATE
        MERMAN_MODULE_NAME="${PROJECT_NAME}"
)

# =========================
# 构建后同步头文件到 common/include
# 注意：
# .so 已经直接输出到 ${MERMAN_COMMON}/lib
# 所以这里不需要再 copy lib
# =========================
add_custom_command(
    TARGET merman_communication
    POST_BUILD

    COMMAND ${CMAKE_COMMAND} -E make_directory
            ${MERMAN_COMMON}/include/merman_communication_interface

    COMMAND ${CMAKE_COMMAND} -E copy_directory
            ${CMAKE_CURRENT_SOURCE_DIR}/include/merman_communication_interface
            ${MERMAN_COMMON}/include/merman_communication_interface
)