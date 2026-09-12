// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from merman_ros2_types:msg/PoseArrivedData.idl
// generated code does not contain a copyright notice

#ifndef MERMAN_ROS2_TYPES__MSG__DETAIL__POSE_ARRIVED_DATA__BUILDER_HPP_
#define MERMAN_ROS2_TYPES__MSG__DETAIL__POSE_ARRIVED_DATA__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "merman_ros2_types/msg/detail/pose_arrived_data__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace merman_ros2_types
{

namespace msg
{

namespace builder
{

class Init_PoseArrivedData_play_audio
{
public:
  explicit Init_PoseArrivedData_play_audio(::merman_ros2_types::msg::PoseArrivedData & msg)
  : msg_(msg)
  {}
  ::merman_ros2_types::msg::PoseArrivedData play_audio(::merman_ros2_types::msg::PoseArrivedData::_play_audio_type arg)
  {
    msg_.play_audio = std::move(arg);
    return std::move(msg_);
  }

private:
  ::merman_ros2_types::msg::PoseArrivedData msg_;
};

class Init_PoseArrivedData_pose_id
{
public:
  Init_PoseArrivedData_pose_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PoseArrivedData_play_audio pose_id(::merman_ros2_types::msg::PoseArrivedData::_pose_id_type arg)
  {
    msg_.pose_id = std::move(arg);
    return Init_PoseArrivedData_play_audio(msg_);
  }

private:
  ::merman_ros2_types::msg::PoseArrivedData msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::merman_ros2_types::msg::PoseArrivedData>()
{
  return merman_ros2_types::msg::builder::Init_PoseArrivedData_pose_id();
}

}  // namespace merman_ros2_types

#endif  // MERMAN_ROS2_TYPES__MSG__DETAIL__POSE_ARRIVED_DATA__BUILDER_HPP_
