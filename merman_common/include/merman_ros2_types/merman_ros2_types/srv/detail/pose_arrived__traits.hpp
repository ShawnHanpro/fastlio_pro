// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from merman_ros2_types:srv/PoseArrived.idl
// generated code does not contain a copyright notice

#ifndef MERMAN_ROS2_TYPES__SRV__DETAIL__POSE_ARRIVED__TRAITS_HPP_
#define MERMAN_ROS2_TYPES__SRV__DETAIL__POSE_ARRIVED__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "merman_ros2_types/srv/detail/pose_arrived__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace merman_ros2_types
{

namespace srv
{

inline void to_flow_style_yaml(
  const PoseArrived_Request & msg,
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
  const PoseArrived_Request & msg,
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

inline std::string to_yaml(const PoseArrived_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace merman_ros2_types

namespace rosidl_generator_traits
{

[[deprecated("use merman_ros2_types::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const merman_ros2_types::srv::PoseArrived_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  merman_ros2_types::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use merman_ros2_types::srv::to_yaml() instead")]]
inline std::string to_yaml(const merman_ros2_types::srv::PoseArrived_Request & msg)
{
  return merman_ros2_types::srv::to_yaml(msg);
}

template<>
inline const char * data_type<merman_ros2_types::srv::PoseArrived_Request>()
{
  return "merman_ros2_types::srv::PoseArrived_Request";
}

template<>
inline const char * name<merman_ros2_types::srv::PoseArrived_Request>()
{
  return "merman_ros2_types/srv/PoseArrived_Request";
}

template<>
struct has_fixed_size<merman_ros2_types::srv::PoseArrived_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<merman_ros2_types::srv::PoseArrived_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<merman_ros2_types::srv::PoseArrived_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace merman_ros2_types
{

namespace srv
{

inline void to_flow_style_yaml(
  const PoseArrived_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << ", ";
  }

  // member: message
  {
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const PoseArrived_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: success
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << "\n";
  }

  // member: message
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const PoseArrived_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace merman_ros2_types

namespace rosidl_generator_traits
{

[[deprecated("use merman_ros2_types::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const merman_ros2_types::srv::PoseArrived_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  merman_ros2_types::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use merman_ros2_types::srv::to_yaml() instead")]]
inline std::string to_yaml(const merman_ros2_types::srv::PoseArrived_Response & msg)
{
  return merman_ros2_types::srv::to_yaml(msg);
}

template<>
inline const char * data_type<merman_ros2_types::srv::PoseArrived_Response>()
{
  return "merman_ros2_types::srv::PoseArrived_Response";
}

template<>
inline const char * name<merman_ros2_types::srv::PoseArrived_Response>()
{
  return "merman_ros2_types/srv/PoseArrived_Response";
}

template<>
struct has_fixed_size<merman_ros2_types::srv::PoseArrived_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<merman_ros2_types::srv::PoseArrived_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<merman_ros2_types::srv::PoseArrived_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<merman_ros2_types::srv::PoseArrived>()
{
  return "merman_ros2_types::srv::PoseArrived";
}

template<>
inline const char * name<merman_ros2_types::srv::PoseArrived>()
{
  return "merman_ros2_types/srv/PoseArrived";
}

template<>
struct has_fixed_size<merman_ros2_types::srv::PoseArrived>
  : std::integral_constant<
    bool,
    has_fixed_size<merman_ros2_types::srv::PoseArrived_Request>::value &&
    has_fixed_size<merman_ros2_types::srv::PoseArrived_Response>::value
  >
{
};

template<>
struct has_bounded_size<merman_ros2_types::srv::PoseArrived>
  : std::integral_constant<
    bool,
    has_bounded_size<merman_ros2_types::srv::PoseArrived_Request>::value &&
    has_bounded_size<merman_ros2_types::srv::PoseArrived_Response>::value
  >
{
};

template<>
struct is_service<merman_ros2_types::srv::PoseArrived>
  : std::true_type
{
};

template<>
struct is_service_request<merman_ros2_types::srv::PoseArrived_Request>
  : std::true_type
{
};

template<>
struct is_service_response<merman_ros2_types::srv::PoseArrived_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // MERMAN_ROS2_TYPES__SRV__DETAIL__POSE_ARRIVED__TRAITS_HPP_
