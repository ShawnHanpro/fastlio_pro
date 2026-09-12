# merman_toolchain.cmake
string(TOLOWER "${CMAKE_SYSTEM_PROCESSOR}" ARCH_LOWER)

if(ARCH_LOWER MATCHES "aarch64|arm64")
    set(MERMAN_ARCH arm64)
elseif(ARCH_LOWER MATCHES "x86_64|amd64")
    set(MERMAN_ARCH x86_64)
else()
    message(FATAL_ERROR
        "Unsupported architecture: ${CMAKE_SYSTEM_PROCESSOR}")
endif()

# merman_pro 根目录
get_filename_component(
    MERMAN_ROOT
    "${CMAKE_CURRENT_LIST_DIR}/../.."
    ABSOLUTE
)

set(THIRD_PARTY_DIR
    "${MERMAN_ROOT}/merman_common/third_party/${MERMAN_ARCH}")
set(DRIVERS_PARTY_DIR
    "${MERMAN_ROOT}/merman_common/drivers/${MERMAN_ARCH}")

message(STATUS "Architecture : ${MERMAN_ARCH}")
message(STATUS "Third Party  : ${THIRD_PARTY_DIR}")



# set(MERMAN_COMMON_DIR
#     "$ENV{HOME}/merman_pro/merman_common")

# set(THIRD_PARTY_DIR "${MERMAN_COMMON_DIR}/third_party")

# # 统一转换为小写便于比较
# string(TOLOWER "${CMAKE_SYSTEM_PROCESSOR}" ARCH_LOWER)

# if(ARCH_LOWER MATCHES "arm|aarch64")
#     set(THIRD_PARTY_DIR "${THIRD_PARTY_DIR}/arm64")
# elseif(ARCH_LOWER MATCHES "x86|x86_64|x64|amd64")
#     set(THIRD_PARTY_DIR "${THIRD_PARTY_DIR}/x86_64")
# else()
#     message(FATAL_ERROR "Unsupported architecture: ${CMAKE_SYSTEM_PROCESSOR}")
# endif()

# message(STATUS "Third party dir: ${THIRD_PARTY_DIR}")

# set(THIRD_PARTY_DIR
#     "${MERMAN_COMMON_DIR}/third_party")



# CMake 搜索路径
list(PREPEND CMAKE_PREFIX_PATH
    ${THIRD_PARTY_DIR})

set(LIBPOINTMATCHER_DIR 
    ${THIRD_PARTY_DIR}/libpointmatcher/share/libpointmatcher/cmake
)

set(Eigen3_DIR
    ${THIRD_PARTY_DIR}/eigen/share/eigen3/cmake)

set(OpenCV_DIR
    ${THIRD_PARTY_DIR}/opencv/lib/cmake/opencv4)

set(PCL_DIR
    ${THIRD_PARTY_DIR}/pcl/share/pcl-1.12)

set(g2o_DIR
    ${THIRD_PARTY_DIR}/g2o/lib/cmake/g2o)

set(LIVOX_SDK_ROOT 
    ${DRIVERS_PARTY_DIR}/Livox-SDK2) 

set(Qhull_DIR
    ${THIRD_PARTY_DIR}/qhull/lib/cmake/Qhull)

list(PREPEND CMAKE_PREFIX_PATH
    ${THIRD_PARTY_DIR}/qhull
    ${THIRD_PARTY_DIR}/g2o
)


# 运行时库路径（RPATH）
set(CMAKE_BUILD_RPATH
    ${THIRD_PARTY_DIR}/boost/lib
    ${THIRD_PARTY_DIR}/opencv/lib
    ${THIRD_PARTY_DIR}/flann/lib
    ${THIRD_PARTY_DIR}/g2o/lib
    ${THIRD_PARTY_DIR}/sqlite/lib
    ${THIRD_PARTY_DIR}/pcl/lib
    ${THIRD_PARTY_DIR}/libpointmatcher/lib
    ${LIVOX_SDK_ROOT}/lib 
    ${THIRD_PARTY_DIR}/qhull/lib
)

set(CMAKE_INSTALL_RPATH
    ${THIRD_PARTY_DIR}/boost/lib
    ${THIRD_PARTY_DIR}/opencv/lib
    ${THIRD_PARTY_DIR}/flann/lib
    ${THIRD_PARTY_DIR}/g2o/lib
    ${THIRD_PARTY_DIR}/sqlite/lib
    ${THIRD_PARTY_DIR}/pcl/lib
    ${THIRD_PARTY_DIR}/libpointmatcher/lib
    ${LIVOX_SDK_ROOT}/lib 
    ${THIRD_PARTY_DIR}/qhull/lib
)

set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)