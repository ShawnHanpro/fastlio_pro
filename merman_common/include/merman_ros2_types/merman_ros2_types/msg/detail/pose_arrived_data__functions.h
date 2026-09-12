// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from merman_ros2_types:msg/PoseArrivedData.idl
// generated code does not contain a copyright notice

#ifndef MERMAN_ROS2_TYPES__MSG__DETAIL__POSE_ARRIVED_DATA__FUNCTIONS_H_
#define MERMAN_ROS2_TYPES__MSG__DETAIL__POSE_ARRIVED_DATA__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "merman_ros2_types/msg/rosidl_generator_c__visibility_control.h"

#include "merman_ros2_types/msg/detail/pose_arrived_data__struct.h"

/// Initialize msg/PoseArrivedData message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * merman_ros2_types__msg__PoseArrivedData
 * )) before or use
 * merman_ros2_types__msg__PoseArrivedData__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_merman_ros2_types
bool
merman_ros2_types__msg__PoseArrivedData__init(merman_ros2_types__msg__PoseArrivedData * msg);

/// Finalize msg/PoseArrivedData message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_merman_ros2_types
void
merman_ros2_types__msg__PoseArrivedData__fini(merman_ros2_types__msg__PoseArrivedData * msg);

/// Create msg/PoseArrivedData message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * merman_ros2_types__msg__PoseArrivedData__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_merman_ros2_types
merman_ros2_types__msg__PoseArrivedData *
merman_ros2_types__msg__PoseArrivedData__create();

/// Destroy msg/PoseArrivedData message.
/**
 * It calls
 * merman_ros2_types__msg__PoseArrivedData__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_merman_ros2_types
void
merman_ros2_types__msg__PoseArrivedData__destroy(merman_ros2_types__msg__PoseArrivedData * msg);

/// Check for msg/PoseArrivedData message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_merman_ros2_types
bool
merman_ros2_types__msg__PoseArrivedData__are_equal(const merman_ros2_types__msg__PoseArrivedData * lhs, const merman_ros2_types__msg__PoseArrivedData * rhs);

/// Copy a msg/PoseArrivedData message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_merman_ros2_types
bool
merman_ros2_types__msg__PoseArrivedData__copy(
  const merman_ros2_types__msg__PoseArrivedData * input,
  merman_ros2_types__msg__PoseArrivedData * output);

/// Initialize array of msg/PoseArrivedData messages.
/**
 * It allocates the memory for the number of elements and calls
 * merman_ros2_types__msg__PoseArrivedData__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_merman_ros2_types
bool
merman_ros2_types__msg__PoseArrivedData__Sequence__init(merman_ros2_types__msg__PoseArrivedData__Sequence * array, size_t size);

/// Finalize array of msg/PoseArrivedData messages.
/**
 * It calls
 * merman_ros2_types__msg__PoseArrivedData__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_merman_ros2_types
void
merman_ros2_types__msg__PoseArrivedData__Sequence__fini(merman_ros2_types__msg__PoseArrivedData__Sequence * array);

/// Create array of msg/PoseArrivedData messages.
/**
 * It allocates the memory for the array and calls
 * merman_ros2_types__msg__PoseArrivedData__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_merman_ros2_types
merman_ros2_types__msg__PoseArrivedData__Sequence *
merman_ros2_types__msg__PoseArrivedData__Sequence__create(size_t size);

/// Destroy array of msg/PoseArrivedData messages.
/**
 * It calls
 * merman_ros2_types__msg__PoseArrivedData__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_merman_ros2_types
void
merman_ros2_types__msg__PoseArrivedData__Sequence__destroy(merman_ros2_types__msg__PoseArrivedData__Sequence * array);

/// Check for msg/PoseArrivedData message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_merman_ros2_types
bool
merman_ros2_types__msg__PoseArrivedData__Sequence__are_equal(const merman_ros2_types__msg__PoseArrivedData__Sequence * lhs, const merman_ros2_types__msg__PoseArrivedData__Sequence * rhs);

/// Copy an array of msg/PoseArrivedData messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_merman_ros2_types
bool
merman_ros2_types__msg__PoseArrivedData__Sequence__copy(
  const merman_ros2_types__msg__PoseArrivedData__Sequence * input,
  merman_ros2_types__msg__PoseArrivedData__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // MERMAN_ROS2_TYPES__MSG__DETAIL__POSE_ARRIVED_DATA__FUNCTIONS_H_
