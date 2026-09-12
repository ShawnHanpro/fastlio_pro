// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from merman_ros2_types:msg/PoseArrivedData.idl
// generated code does not contain a copyright notice
#include "merman_ros2_types/msg/detail/pose_arrived_data__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
merman_ros2_types__msg__PoseArrivedData__init(merman_ros2_types__msg__PoseArrivedData * msg)
{
  if (!msg) {
    return false;
  }
  // pose_id
  // play_audio
  return true;
}

void
merman_ros2_types__msg__PoseArrivedData__fini(merman_ros2_types__msg__PoseArrivedData * msg)
{
  if (!msg) {
    return;
  }
  // pose_id
  // play_audio
}

bool
merman_ros2_types__msg__PoseArrivedData__are_equal(const merman_ros2_types__msg__PoseArrivedData * lhs, const merman_ros2_types__msg__PoseArrivedData * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // pose_id
  if (lhs->pose_id != rhs->pose_id) {
    return false;
  }
  // play_audio
  if (lhs->play_audio != rhs->play_audio) {
    return false;
  }
  return true;
}

bool
merman_ros2_types__msg__PoseArrivedData__copy(
  const merman_ros2_types__msg__PoseArrivedData * input,
  merman_ros2_types__msg__PoseArrivedData * output)
{
  if (!input || !output) {
    return false;
  }
  // pose_id
  output->pose_id = input->pose_id;
  // play_audio
  output->play_audio = input->play_audio;
  return true;
}

merman_ros2_types__msg__PoseArrivedData *
merman_ros2_types__msg__PoseArrivedData__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  merman_ros2_types__msg__PoseArrivedData * msg = (merman_ros2_types__msg__PoseArrivedData *)allocator.allocate(sizeof(merman_ros2_types__msg__PoseArrivedData), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(merman_ros2_types__msg__PoseArrivedData));
  bool success = merman_ros2_types__msg__PoseArrivedData__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
merman_ros2_types__msg__PoseArrivedData__destroy(merman_ros2_types__msg__PoseArrivedData * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    merman_ros2_types__msg__PoseArrivedData__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
merman_ros2_types__msg__PoseArrivedData__Sequence__init(merman_ros2_types__msg__PoseArrivedData__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  merman_ros2_types__msg__PoseArrivedData * data = NULL;

  if (size) {
    data = (merman_ros2_types__msg__PoseArrivedData *)allocator.zero_allocate(size, sizeof(merman_ros2_types__msg__PoseArrivedData), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = merman_ros2_types__msg__PoseArrivedData__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        merman_ros2_types__msg__PoseArrivedData__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
merman_ros2_types__msg__PoseArrivedData__Sequence__fini(merman_ros2_types__msg__PoseArrivedData__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      merman_ros2_types__msg__PoseArrivedData__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

merman_ros2_types__msg__PoseArrivedData__Sequence *
merman_ros2_types__msg__PoseArrivedData__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  merman_ros2_types__msg__PoseArrivedData__Sequence * array = (merman_ros2_types__msg__PoseArrivedData__Sequence *)allocator.allocate(sizeof(merman_ros2_types__msg__PoseArrivedData__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = merman_ros2_types__msg__PoseArrivedData__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
merman_ros2_types__msg__PoseArrivedData__Sequence__destroy(merman_ros2_types__msg__PoseArrivedData__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    merman_ros2_types__msg__PoseArrivedData__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
merman_ros2_types__msg__PoseArrivedData__Sequence__are_equal(const merman_ros2_types__msg__PoseArrivedData__Sequence * lhs, const merman_ros2_types__msg__PoseArrivedData__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!merman_ros2_types__msg__PoseArrivedData__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
merman_ros2_types__msg__PoseArrivedData__Sequence__copy(
  const merman_ros2_types__msg__PoseArrivedData__Sequence * input,
  merman_ros2_types__msg__PoseArrivedData__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(merman_ros2_types__msg__PoseArrivedData);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    merman_ros2_types__msg__PoseArrivedData * data =
      (merman_ros2_types__msg__PoseArrivedData *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!merman_ros2_types__msg__PoseArrivedData__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          merman_ros2_types__msg__PoseArrivedData__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!merman_ros2_types__msg__PoseArrivedData__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
