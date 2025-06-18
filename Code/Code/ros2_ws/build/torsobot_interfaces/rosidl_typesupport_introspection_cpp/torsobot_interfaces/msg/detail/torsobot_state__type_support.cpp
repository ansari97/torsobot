// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from torsobot_interfaces:msg/TorsobotState.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "torsobot_interfaces/msg/detail/torsobot_state__functions.h"
#include "torsobot_interfaces/msg/detail/torsobot_state__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace torsobot_interfaces
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void TorsobotState_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) torsobot_interfaces::msg::TorsobotState(_init);
}

void TorsobotState_fini_function(void * message_memory)
{
  auto typed_message = static_cast<torsobot_interfaces::msg::TorsobotState *>(message_memory);
  typed_message->~TorsobotState();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember TorsobotState_message_member_array[4] = {
  {
    "torso_pitch",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(torsobot_interfaces::msg::TorsobotState, torso_pitch),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "torso_pitch_rate",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(torsobot_interfaces::msg::TorsobotState, torso_pitch_rate),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "motor_pos",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(torsobot_interfaces::msg::TorsobotState, motor_pos),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "motor_vel",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(torsobot_interfaces::msg::TorsobotState, motor_vel),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers TorsobotState_message_members = {
  "torsobot_interfaces::msg",  // message namespace
  "TorsobotState",  // message name
  4,  // number of fields
  sizeof(torsobot_interfaces::msg::TorsobotState),
  false,  // has_any_key_member_
  TorsobotState_message_member_array,  // message members
  TorsobotState_init_function,  // function to initialize message memory (memory has to be allocated)
  TorsobotState_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t TorsobotState_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &TorsobotState_message_members,
  get_message_typesupport_handle_function,
  &torsobot_interfaces__msg__TorsobotState__get_type_hash,
  &torsobot_interfaces__msg__TorsobotState__get_type_description,
  &torsobot_interfaces__msg__TorsobotState__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace torsobot_interfaces


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<torsobot_interfaces::msg::TorsobotState>()
{
  return &::torsobot_interfaces::msg::rosidl_typesupport_introspection_cpp::TorsobotState_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, torsobot_interfaces, msg, TorsobotState)() {
  return &::torsobot_interfaces::msg::rosidl_typesupport_introspection_cpp::TorsobotState_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
