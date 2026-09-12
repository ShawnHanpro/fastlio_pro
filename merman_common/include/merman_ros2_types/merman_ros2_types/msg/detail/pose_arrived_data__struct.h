// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from merman_ros2_types:msg/PoseArrivedData.idl
// generated code does not contain a copyright notice

#ifndef MERMAN_ROS2_TYPES__MSG__DETAIL__POSE_ARRIVED_DATA__STRUCT_H_
#define MERMAN_ROS2_TYPES__MSG__DETAIL__POSE_ARRIVED_DATA__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/PoseArrivedData in the package merman_ros2_types.
typedef struct merman_ros2_types__msg__PoseArrivedData
{
  int32_t pose_id;
  bool play_audio;
} merman_ros2_types__msg__PoseArrivedData;

// Struct for a sequence of merman_ros2_types__msg__PoseArrivedData.
typedef struct merman_ros2_types__msg__PoseArrivedData__Sequence
{
  merman_ros2_types__msg__PoseArrivedData * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} merman_ros2_types__msg__PoseArrivedData__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MERMAN_ROS2_TYPES__MSG__DETAIL__POSE_ARRIVED_DATA__STRUCT_H_
