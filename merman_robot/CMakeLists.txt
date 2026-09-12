cmake_minimum_required(VERSION 3.22)

project(merman_robot LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(MERMAN_COMMON
    ${CMAKE_CURRENT_SOURCE_DIR}/../merman_common
)

# =========================
# 找 control
# =========================
find_library(
    MERMAN_CONTROL_LIB
    NAMES merman_control
    PATHS "${MERMAN_COMMON}/lib"
    REQUIRED
    NO_DEFAULT_PATH
)

# =========================
# 找 hardware
# =========================
find_library(
    MERMAN_HARDWARE_LIB
    NAMES merman_hardware
    PATHS "${MERMAN_COMMON}/lib"
    REQUIRED
    NO_DEFAULT_PATH
)

# =========================
# 可执行程序
# =========================
add_executable(
    merman_robot
    src/main/merman_robot.cpp
    src/main/boot_init.cpp
)

target_include_directories(
    merman_robot
    PRIVATE
        ${MERMAN_COMMON}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${MERMAN_COMMON}/types
        ${MERMAN_COMMON}/third_party/arm64/include/eigen3
)

# =========================
# 注入模块名供 MermanLogger 自动识别
# =========================
target_compile_definitions(
    merman_robot
    PRIVATE
        MERMAN_MODULE_NAME="${PROJECT_NAME}"
)

target_link_libraries(
    merman_robot
    PRIVATE
        ${MERMAN_CONTROL_LIB}
        ${MERMAN_HARDWARE_LIB}
        spdlog
        fmt
)

# =========================
# 关键设置
#
# 默认情况下 GNU ld 通常生成 DT_RUNPATH。
# --disable-new-dtags 让 -rpath 生成 DT_RPATH。
#
# DT_RPATH 能用于间接依赖和 dlopen 动态加载的库。
# =========================
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    target_link_options(
        merman_robot
        PRIVATE
            "LINKER:--disable-new-dtags"
    )
endif()

set_target_properties(
    merman_robot
    PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY "${MERMAN_COMMON}/bin"

        BUILD_RPATH "\$ORIGIN/../lib"
        INSTALL_RPATH "\$ORIGIN/../lib"

        BUILD_WITH_INSTALL_RPATH TRUE
        INSTALL_RPATH_USE_LINK_PATH FALSE
)

# =========================
# 可选：只解决链接阶段的间接库搜索
# 不影响运行时，但保留没有坏处
# =========================
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    target_link_options(
        merman_robot
        PRIVATE
            "LINKER:-rpath-link,${MERMAN_COMMON}/lib"
    )
endif()










# cmake_minimum_required(VERSION 3.22)

# project(merman_robot)

# set(MERMAN_COMMON
#     ${CMAKE_CURRENT_SOURCE_DIR}/../merman_common
# )

# find_library(
#     MERMAN_CONTROL_LIB
#     merman_control
#     PATHS
#         ${MERMAN_COMMON}/lib
#     REQUIRED
#     NO_DEFAULT_PATH
# )

# find_library(
#     MERMAN_HARDWARE_LIB
#     merman_hardware
#     PATHS
#         ${MERMAN_COMMON}/lib
#     REQUIRED
#     NO_DEFAULT_PATH
# )

# add_executable(merman_robot
#     src/main/merman_robot.cpp
#     src/main/boot_init.cpp
# )

# target_include_directories(merman_robot
#     PRIVATE
#         ${MERMAN_COMMON}/include
#         ${CMAKE_CURRENT_SOURCE_DIR}/include
#         ${MERMAN_COMMON}/types
#         ${MERMAN_COMMON}/third_party/arm64/include/eigen3
# )

# target_link_libraries(merman_robot
#     PRIVATE
#         ${MERMAN_CONTROL_LIB}
#         ${MERMAN_HARDWARE_LIB}
# )

# set_target_properties(merman_robot PROPERTIES
#     RUNTIME_OUTPUT_DIRECTORY "${MERMAN_COMMON}/bin"

#     BUILD_RPATH "\$ORIGIN/../lib"
#     INSTALL_RPATH "\$ORIGIN/../lib"
#     BUILD_WITH_INSTALL_RPATH TRUE
# )