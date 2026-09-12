// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from merman_ros2_types:msg/PoseArrivedData.idl
// generated code does not contain a copyright notice

#ifndef MERMAN_ROS2_TYPES__MSG__DETAIL__POSE_ARRIVED_DATA__STRUCT_HPP_
#define MERMAN_ROS2_TYPES__MSG__DETAIL__POSE_ARRIVED_DATA__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__merman_ros2_types__msg__PoseArrivedData __attribute__((deprecated))
#else
# define DEPRECATED__merman_ros2_types__msg__PoseArrivedData __declspec(deprecated)
#endif

namespace merman_ros2_types
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct PoseArrivedData_
{
  using Type = PoseArrivedData_<ContainerAllocator>;

  explicit PoseArrivedData_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->pose_id = 0l;
      this->play_audio = false;
    }
  }

  explicit PoseArrivedData_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->pose_id = 0l;
      this->play_audio = false;
    }
  }

  // field types and members
  using _pose_id_type =
    int32_t;
  _pose_id_type pose_id;
  using _play_audio_type =
    bool;
  _play_audio_type play_audio;

  // setters for named parameter idiom
  Type & set__pose_id(
    const int32_t & _arg)
  {
    this->pose_id = _arg;
    return *this;
  }
  Type & set__play_audio(
    const bool & _arg)
  {
    this->play_audio = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    merman_ros2_types::msg::PoseArrivedData_<ContainerAllocator> *;
  using ConstRawPtr =
    const merman_ros2_types::msg::PoseArrivedData_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<merman_ros2_types::msg::PoseArrivedData_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<merman_ros2_types::msg::PoseArrivedData_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      merman_ros2_types::msg::PoseArrivedData_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<merman_ros2_types::msg::PoseArrivedData_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      merman_ros2_types::msg::PoseArrivedData_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<merman_ros2_types::msg::PoseArrivedData_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<merman_ros2_types::msg::PoseArrivedData_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<merman_ros2_types::msg::PoseArrivedData_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__merman_ros2_types__msg__PoseArrivedData
    std::shared_ptr<merman_ros2_types::msg::PoseArrivedData_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__merman_ros2_types__msg__PoseArrivedData
    std::shared_ptr<merman_ros2_types::msg::PoseArrivedData_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PoseArrivedData_ & other) const
  {
    if (this->pose_id != other.pose_id) {
      return false;
    }
    if (this->play_audio != other.play_audio) {
      return false;
    }
    return true;
  }
  bool operator!=(const PoseArrivedData_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PoseArrivedData_

// alias to use template instance with default allocator
using PoseArrivedData =
  merman_ros2_types::msg::PoseArrivedData_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace merman_ros2_types

#endif  // MERMAN_ROS2_TYPES__MSG__DETAIL__POSE_ARRIVED_DATA__STRUCT_HPP_
