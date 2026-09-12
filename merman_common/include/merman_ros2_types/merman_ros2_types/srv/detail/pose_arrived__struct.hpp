// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from merman_ros2_types:srv/PoseArrived.idl
// generated code does not contain a copyright notice

#ifndef MERMAN_ROS2_TYPES__SRV__DETAIL__POSE_ARRIVED__STRUCT_HPP_
#define MERMAN_ROS2_TYPES__SRV__DETAIL__POSE_ARRIVED__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__merman_ros2_types__srv__PoseArrived_Request __attribute__((deprecated))
#else
# define DEPRECATED__merman_ros2_types__srv__PoseArrived_Request __declspec(deprecated)
#endif

namespace merman_ros2_types
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct PoseArrived_Request_
{
  using Type = PoseArrived_Request_<ContainerAllocator>;

  explicit PoseArrived_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->pose_id = 0l;
      this->play_audio = false;
    }
  }

  explicit PoseArrived_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    merman_ros2_types::srv::PoseArrived_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const merman_ros2_types::srv::PoseArrived_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<merman_ros2_types::srv::PoseArrived_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<merman_ros2_types::srv::PoseArrived_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      merman_ros2_types::srv::PoseArrived_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<merman_ros2_types::srv::PoseArrived_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      merman_ros2_types::srv::PoseArrived_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<merman_ros2_types::srv::PoseArrived_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<merman_ros2_types::srv::PoseArrived_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<merman_ros2_types::srv::PoseArrived_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__merman_ros2_types__srv__PoseArrived_Request
    std::shared_ptr<merman_ros2_types::srv::PoseArrived_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__merman_ros2_types__srv__PoseArrived_Request
    std::shared_ptr<merman_ros2_types::srv::PoseArrived_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PoseArrived_Request_ & other) const
  {
    if (this->pose_id != other.pose_id) {
      return false;
    }
    if (this->play_audio != other.play_audio) {
      return false;
    }
    return true;
  }
  bool operator!=(const PoseArrived_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PoseArrived_Request_

// alias to use template instance with default allocator
using PoseArrived_Request =
  merman_ros2_types::srv::PoseArrived_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace merman_ros2_types


#ifndef _WIN32
# define DEPRECATED__merman_ros2_types__srv__PoseArrived_Response __attribute__((deprecated))
#else
# define DEPRECATED__merman_ros2_types__srv__PoseArrived_Response __declspec(deprecated)
#endif

namespace merman_ros2_types
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct PoseArrived_Response_
{
  using Type = PoseArrived_Response_<ContainerAllocator>;

  explicit PoseArrived_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  explicit PoseArrived_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : message(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;
  using _message_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _message_type message;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }
  Type & set__message(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->message = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    merman_ros2_types::srv::PoseArrived_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const merman_ros2_types::srv::PoseArrived_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<merman_ros2_types::srv::PoseArrived_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<merman_ros2_types::srv::PoseArrived_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      merman_ros2_types::srv::PoseArrived_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<merman_ros2_types::srv::PoseArrived_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      merman_ros2_types::srv::PoseArrived_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<merman_ros2_types::srv::PoseArrived_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<merman_ros2_types::srv::PoseArrived_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<merman_ros2_types::srv::PoseArrived_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__merman_ros2_types__srv__PoseArrived_Response
    std::shared_ptr<merman_ros2_types::srv::PoseArrived_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__merman_ros2_types__srv__PoseArrived_Response
    std::shared_ptr<merman_ros2_types::srv::PoseArrived_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PoseArrived_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->message != other.message) {
      return false;
    }
    return true;
  }
  bool operator!=(const PoseArrived_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PoseArrived_Response_

// alias to use template instance with default allocator
using PoseArrived_Response =
  merman_ros2_types::srv::PoseArrived_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace merman_ros2_types

namespace merman_ros2_types
{

namespace srv
{

struct PoseArrived
{
  using Request = merman_ros2_types::srv::PoseArrived_Request;
  using Response = merman_ros2_types::srv::PoseArrived_Response;
};

}  // namespace srv

}  // namespace merman_ros2_types

#endif  // MERMAN_ROS2_TYPES__SRV__DETAIL__POSE_ARRIVED__STRUCT_HPP_
