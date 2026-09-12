cmake_minimum_required(VERSION 3.22)

project(merman_hardware LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# =========================
# common 路径
# =========================
set(MERMAN_COMMON
    ${CMAKE_CURRENT_SOURCE_DIR}/../merman_common
)

# =========================
# 线程库
# =========================
find_package(Threads REQUIRED)

# =========================
# 源文件
# =========================
file(GLOB_RECURSE HARDWARE_SRC
    CONFIGURE_DEPENDS
    src/*.cpp
)

# =========================
# 生成动态库
# =========================
add_library(
    merman_hardware
    SHARED
    ${HARDWARE_SRC}
)

# =========================
# 输出路径 + RPATH
# =========================
set_target_properties(merman_hardware PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${MERMAN_COMMON}/lib"
    ARCHIVE_OUTPUT_DIRECTORY "${MERMAN_COMMON}/lib"
    RUNTIME_OUTPUT_DIRECTORY "${MERMAN_COMMON}/bin"

    BUILD_RPATH "\$ORIGIN"
    INSTALL_RPATH "\$ORIGIN"
    BUILD_WITH_INSTALL_RPATH TRUE
)

# =========================
# 配置文件路径
# =========================
target_compile_definitions(
    merman_hardware
    PRIVATE
        MERMAN_COMMON_CONFIG_DIR="${MERMAN_COMMON}/config"
        MERMAN_MODULE_NAME="${PROJECT_NAME}"
)

# =========================
# 头文件路径
# =========================
target_include_directories(
    merman_hardware
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/include/merman_hardware_interface

    PRIVATE
        ${MERMAN_COMMON}/include
        ${MERMAN_COMMON}/types
        ${MERMAN_COMMON}/third_party/arm64/include/eigen3
)

# =========================
# 从 common 找 control lib
# =========================
find_library(
    MERMAN_CONTROL_LIB
    merman_control
    PATHS ${MERMAN_COMMON}/lib
    REQUIRED
)

if(NOT MERMAN_CONTROL_LIB)
    message(FATAL_ERROR "Cannot find merman_control in ${MERMAN_COMMON}/lib")
endif()

# =========================
# 链接 control + pthread
# =========================
target_link_libraries(
    merman_hardware
    PUBLIC
        ${MERMAN_CONTROL_LIB}
        Threads::Threads
)

# =========================
# 构建后同步头文件到 common/include
# 注意：
# .so 已经直接输出到 ${MERMAN_COMMON}/lib
# 所以这里不需要再 copy lib
# =========================
add_custom_command(
    TARGET merman_hardware
    POST_BUILD

    COMMAND ${CMAKE_COMMAND} -E make_directory
            ${MERMAN_COMMON}/include/merman_hardware_interface

    COMMAND ${CMAKE_COMMAND} -E copy_directory
            ${CMAKE_CURRENT_SOURCE_DIR}/include/merman_hardware_interface
            ${MERMAN_COMMON}/include/merman_hardware_interface
)