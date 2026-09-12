cmake_minimum_required(VERSION 3.22)

project(merman_slam)

# include(${CMAKE_CURRENT_SOURCE_DIR}/../merman_common/cmake/third_party.cmake)
set(MERMAN_COMMON
    ${CMAKE_SOURCE_DIR}/../merman_common
)

file(GLOB_RECURSE SRC
    src/*.cpp
)

add_library(
    merman_slam
    SHARED
    ${SRC}
)

# =========================
# 注入模块名供 MermanLogger 自动识别
# =========================
target_compile_definitions(
    merman_slam
    PRIVATE
        MERMAN_MODULE_NAME="${PROJECT_NAME}"
)

target_include_directories(
    merman_slam
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${MERMAN_COMMON}/include
        ${MERMAN_COMMON}/types
        ${MERMAN_COMMON}/third_party/arm64/include/eigen3
        # ${MERMAN_INCLUDE_DIRS}
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include/merman_slam_interface
)

# target_link_directories(merman_slam PRIVATE
#     ${MERMAN_LIBRARY_DIRS}
# )

# set(THIRD_PARTY
#     ${THIRD_PARTY_DIR}/boost/lib
#     ${THIRD_PARTY_DIR}/flann/lib
#     ${THIRD_PARTY_DIR}/g2o/lib
#     ${THIRD_PARTY_DIR}/libnabo/lib
#     ${THIRD_PARTY_DIR}/libpointmatcher/lib
#     ${THIRD_PARTY_DIR}/opencv/lib
#     ${THIRD_PARTY_DIR}/pcl/lib
#     ${THIRD_PARTY_DIR}/sqlite/lib
# )

# set_target_properties(merman_slam PROPERTIES
#     BUILD_RPATH "${THIRD_PARTY}"
#     INSTALL_RPATH "${THIRD_PARTY}"
# )

set(MERMAN_COMMON
    ${CMAKE_SOURCE_DIR}/../merman_common
)

add_custom_command(
    TARGET merman_slam
    POST_BUILD

    COMMAND ${CMAKE_COMMAND} -E make_directory
            ${MERMAN_COMMON}/lib

    COMMAND ${CMAKE_COMMAND} -E make_directory
            ${MERMAN_COMMON}/include

    COMMAND ${CMAKE_COMMAND} -E copy_if_different
            $<TARGET_FILE:merman_slam>
            ${MERMAN_COMMON}/lib/

    COMMAND ${CMAKE_COMMAND} -E copy_directory
            ${CMAKE_CURRENT_SOURCE_DIR}/include/merman_slam_interface
            ${MERMAN_COMMON}/include/merman_slam_interface
)
