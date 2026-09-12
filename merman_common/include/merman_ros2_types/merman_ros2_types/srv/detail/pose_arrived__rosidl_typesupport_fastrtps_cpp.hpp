// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from merman_ros2_types:srv/PoseArrived.idl
// generated code does not contain a copyright notice

#ifndef MERMAN_ROS2_TYPES__SRV__DETAIL__POSE_ARRIVED__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define MERMAN_ROS2_TYPES__SRV__DETAIL__POSE_ARRIVED__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "merman_ros2_types/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "merman_ros2_types/srv/detail/pose_arrived__struct.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include "fastcdr/Cdr.h"

namespace merman_ros2_types
{

namespace srv
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_merman_ros2_types
cdr_serialize(
  const merman_ros2_types::srv::PoseArrived_Request & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_merman_ros2_types
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  merman_ros2_types::srv::PoseArrived_Request & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_merman_ros2_types
get_serialized_size(
  const merman_ros2_types::srv::PoseArrived_Request & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_merman_ros2_types
max_serialized_size_PoseArrived_Request(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace srv

}  // namespace merman_ros2_types

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_merman_ros2_types
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, merman_ros2_types, srv, PoseArrived_Request)();

#ifdef __cplusplus
}
#endif

// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "merman_ros2_types/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
// already included above
// #include "merman_ros2_types/srv/detail/pose_arrived__struct.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// already included above
// #include "fastcdr/Cdr.h"

namespace merman_ros2_types
{

namespace srv
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_merman_ros2_types
cdr_serialize(
  const merman_ros2_types::srv::PoseArrived_Response & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_merman_ros2_types
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  merman_ros2_types::srv::PoseArrived_Response & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_merman_ros2_types
get_serialized_size(
  const merman_ros2_types::srv::PoseArrived_Response & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_merman_ros2_types
max_serialized_size_PoseArrived_Response(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace srv

}  // namespace merman_ros2_types

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_merman_ros2_types
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, merman_ros2_types, srv, PoseArrived_Response)();

#ifdef __cplusplus
}
#endif

#include "rmw/types.h"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "merman_ros2_types/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_merman_ros2_types
const rosidl_service_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, merman_ros2_types, srv, PoseArrived)();

#ifdef __cplusplus
}
#endif

#endif  // MERMAN_ROS2_TYPES__SRV__DETAIL__POSE_ARRIVED__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
