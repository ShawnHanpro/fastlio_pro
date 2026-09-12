// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from merman_ros2_types:srv/PoseArrived.idl
// generated code does not contain a copyright notice

#ifndef MERMAN_ROS2_TYPES__SRV__DETAIL__POSE_ARRIVED__BUILDER_HPP_
#define MERMAN_ROS2_TYPES__SRV__DETAIL__POSE_ARRIVED__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "merman_ros2_types/srv/detail/pose_arrived__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace merman_ros2_types
{

namespace srv
{

namespace builder
{

class Init_PoseArrived_Request_play_audio
{
public:
  explicit Init_PoseArrived_Request_play_audio(::merman_ros2_types::srv::PoseArrived_Request & msg)
  : msg_(msg)
  {}
  ::merman_ros2_types::srv::PoseArrived_Request play_audio(::merman_ros2_types::srv::PoseArrived_Request::_play_audio_type arg)
  {
    msg_.play_audio = std::move(arg);
    return std::move(msg_);
  }

private:
  ::merman_ros2_types::srv::PoseArrived_Request msg_;
};

class Init_PoseArrived_Request_pose_id
{
public:
  Init_PoseArrived_Request_pose_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PoseArrived_Request_play_audio pose_id(::merman_ros2_types::srv::PoseArrived_Request::_pose_id_type arg)
  {
    msg_.pose_id = std::move(arg);
    return Init_PoseArrived_Request_play_audio(msg_);
  }

private:
  ::merman_ros2_types::srv::PoseArrived_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::merman_ros2_types::srv::PoseArrived_Request>()
{
  return merman_ros2_types::srv::builder::Init_PoseArrived_Request_pose_id();
}

}  // namespace merman_ros2_types


namespace merman_ros2_types
{

namespace srv
{

namespace builder
{

class Init_PoseArrived_Response_message
{
public:
  explicit Init_PoseArrived_Response_message(::merman_ros2_types::srv::PoseArrived_Response & msg)
  : msg_(msg)
  {}
  ::merman_ros2_types::srv::PoseArrived_Response message(::merman_ros2_types::srv::PoseArrived_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::merman_ros2_types::srv::PoseArrived_Response msg_;
};

class Init_PoseArrived_Response_success
{
public:
  Init_PoseArrived_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PoseArrived_Response_message success(::merman_ros2_types::srv::PoseArrived_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_PoseArrived_Response_message(msg_);
  }

private:
  ::merman_ros2_types::srv::PoseArrived_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::merman_ros2_types::srv::PoseArrived_Response>()
{
  return merman_ros2_types::srv::builder::Init_PoseArrived_Response_success();
}

}  // namespace merman_ros2_types

#endif  // MERMAN_ROS2_TYPES__SRV__DETAIL__POSE_ARRIVED__BUILDER_HPP_
