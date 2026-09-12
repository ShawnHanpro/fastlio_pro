// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from merman_ros2_types:msg/PoseArrivedData.idl
// generated code does not contain a copyright notice

#ifndef MERMAN_ROS2_TYPES__MSG__DETAIL__POSE_ARRIVED_DATA__TRAITS_HPP_
#define MERMAN_ROS2_TYPES__MSG__DETAIL__POSE_ARRIVED_DATA__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "merman_ros2_types/msg/detail/pose_arrived_data__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace merman_ros2_types
{

namespace msg
{

inline void to_flow_style_yaml(
  const PoseArrivedData & msg,
  std::ostream & out)
{
  out << "{";
  // member: pose_id
  {
    out << "pose_id: ";
    rosidl_generator_traits::value_to_yaml(msg.pose_id, out);
    out << ", ";
  }

  // member: play_audio
  {
    out << "play_audio: ";
    rosidl_generator_traits::value_to_yaml(msg.play_audio, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const PoseArrivedData & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: pose_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pose_id: ";
    rosidl_generator_traits::value_to_yaml(msg.pose_id, out);
    out << "\n";
  }

  // member: play_audio
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "play_audio: ";
    rosidl_generator_traits::value_to_yaml(msg.play_audio, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const PoseArrivedData & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace merman_ros2_types

namespace rosidl_generator_traits
{

[[deprecated("use merman_ros2_types::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const merman_ros2_types::msg::PoseArrivedData & msg,
  std::ostream & out, size_t indentation = 0)
{
  merman_ros2_types::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use merman_ros2_types::msg::to_yaml() instead")]]
inline std::string to_yaml(const merman_ros2_types::msg::PoseArrivedData & msg)
{
  return merman_ros2_types::msg::to_yaml(msg);
}

template<>
inline const char * data_type<merman_ros2_types::msg::PoseArrivedData>()
{
  return "merman_ros2_types::msg::PoseArrivedData";
}

template<>
inline const char * name<merman_ros2_types::msg::PoseArrivedData>()
{
  return "merman_ros2_types/msg/PoseArrivedData";
}

template<>
struct has_fixed_size<merman_ros2_types::msg::PoseArrivedData>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<merman_ros2_types::msg::PoseArrivedData>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<merman_ros2_types::msg::PoseArrivedData>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MERMAN_ROS2_TYPES__MSG__DETAIL__POSE_ARRIVED_DATA__TRAITS_HPP_
