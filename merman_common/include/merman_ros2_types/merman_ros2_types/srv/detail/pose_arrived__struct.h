// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from merman_ros2_types:srv/PoseArrived.idl
// generated code does not contain a copyright notice

#ifndef MERMAN_ROS2_TYPES__SRV__DETAIL__POSE_ARRIVED__STRUCT_H_
#define MERMAN_ROS2_TYPES__SRV__DETAIL__POSE_ARRIVED__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/PoseArrived in the package merman_ros2_types.
typedef struct merman_ros2_types__srv__PoseArrived_Request
{
  int32_t pose_id;
  bool play_audio;
} merman_ros2_types__srv__PoseArrived_Request;

// Struct for a sequence of merman_ros2_types__srv__PoseArrived_Request.
typedef struct merman_ros2_types__srv__PoseArrived_Request__Sequence
{
  merman_ros2_types__srv__PoseArrived_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} merman_ros2_types__srv__PoseArrived_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'message'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/PoseArrived in the package merman_ros2_types.
typedef struct merman_ros2_types__srv__PoseArrived_Response
{
  bool success;
  rosidl_runtime_c__String message;
} merman_ros2_types__srv__PoseArrived_Response;

// Struct for a sequence of merman_ros2_types__srv__PoseArrived_Response.
typedef struct merman_ros2_types__srv__PoseArrived_Response__Sequence
{
  merman_ros2_types__srv__PoseArrived_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} merman_ros2_types__srv__PoseArrived_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MERMAN_ROS2_TYPES__SRV__DETAIL__POSE_ARRIVED__STRUCT_H_
