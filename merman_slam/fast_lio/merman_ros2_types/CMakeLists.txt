cmake_minimum_required(VERSION 3.8)

project(merman_ros2_types)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/PoseArrivedData.msg"
    "srv/PoseArrived.srv"
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()