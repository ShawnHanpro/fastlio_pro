cmake_minimum_required(VERSION 3.22)

project(merman_control)

set(MERMAN_COMMON
    ${CMAKE_CURRENT_SOURCE_DIR}/../merman_common
)

file(GLOB_RECURSE CONTROL_SRC
    CONFIGURE_DEPENDS
    src/*.cpp
)

add_library(
    merman_control
    SHARED
    ${CONTROL_SRC}
)

set_target_properties(merman_control PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${MERMAN_COMMON}/lib"
    BUILD_RPATH "\$ORIGIN"
    INSTALL_RPATH "\$ORIGIN"
    BUILD_WITH_INSTALL_RPATH TRUE
)

target_include_directories(
    merman_control
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${MERMAN_COMMON}/include
        ${MERMAN_COMMON}/types
        ${MERMAN_COMMON}/third_party/arm64/include/eigen3
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include/merman_control_interface
)

# =========================
# 找 slam lib
# =========================
find_library(
    MERMAN_SLAM_LIB
    NAMES merman_slam
    PATHS ${MERMAN_COMMON}/lib
    NO_DEFAULT_PATH
)

if(NOT MERMAN_SLAM_LIB)
    message(FATAL_ERROR "Cannot find merman_slam in ${MERMAN_COMMON}/lib")
endif()

# =========================
# 找 communication lib
# =========================
find_library(
    MERMAN_COMMUNICATION_LIB
    NAMES merman_communication
    PATHS ${MERMAN_COMMON}/lib
    NO_DEFAULT_PATH
)

if(NOT MERMAN_COMMUNICATION_LIB)
    message(FATAL_ERROR "Cannot find merman_communication in ${MERMAN_COMMON}/lib")
endif()

find_package(yaml-cpp REQUIRED)

# =========================
# 链接依赖库
# =========================
target_link_libraries(
    merman_control
    PUBLIC
        ${MERMAN_SLAM_LIB}
        ${MERMAN_COMMUNICATION_LIB}
        yaml-cpp
        pthread
)

# =========================
# 注入模块名供 MermanLogger 自动识别
# =========================
target_compile_definitions(
    merman_control
    PRIVATE
        MERMAN_MODULE_NAME="${PROJECT_NAME}"
)

# =========================
# 安装头文件到 common/include
# =========================
add_custom_command(
    TARGET merman_control
    POST_BUILD

    COMMAND ${CMAKE_COMMAND} -E make_directory
            ${MERMAN_COMMON}/lib

    COMMAND ${CMAKE_COMMAND} -E make_directory
            ${MERMAN_COMMON}/include/

    COMMAND ${CMAKE_COMMAND} -E copy_if_different
            $<TARGET_FILE:merman_control>
            ${MERMAN_COMMON}/lib/

    COMMAND ${CMAKE_COMMAND} -E copy_directory
            ${CMAKE_CURRENT_SOURCE_DIR}/include/merman_control_interface
            ${MERMAN_COMMON}/include/merman_control_interface
)
