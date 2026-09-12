// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from merman_ros2_types:msg/PoseArrivedData.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "merman_ros2_types/msg/detail/pose_arrived_data__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace merman_ros2_types
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void PoseArrivedData_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) merman_ros2_types::msg::PoseArrivedData(_init);
}

void PoseArrivedData_fini_function(void * message_memory)
{
  auto typed_message = static_cast<merman_ros2_types::msg::PoseArrivedData *>(message_memory);
  typed_message->~PoseArrivedData();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember PoseArrivedData_message_member_array[2] = {
  {
    "pose_id",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(merman_ros2_types::msg::PoseArrivedData, pose_id),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "play_audio",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(merman_ros2_types::msg::PoseArrivedData, play_audio),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers PoseArrivedData_message_members = {
  "merman_ros2_types::msg",  // message namespace
  "PoseArrivedData",  // message name
  2,  // number of fields
  sizeof(merman_ros2_types::msg::PoseArrivedData),
  PoseArrivedData_message_member_array,  // message members
  PoseArrivedData_init_function,  // function to initialize message memory (memory has to be allocated)
  PoseArrivedData_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t PoseArrivedData_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &PoseArrivedData_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace merman_ros2_types


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<merman_ros2_types::msg::PoseArrivedData>()
{
  return &::merman_ros2_types::msg::rosidl_typesupport_introspection_cpp::PoseArrivedData_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, merman_ros2_types, msg, PoseArrivedData)() {
  return &::merman_ros2_types::msg::rosidl_typesupport_introspection_cpp::PoseArrivedData_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
