// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from torsobot_interfaces:msg/TorsobotData.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "torsobot_interfaces/msg/detail/torsobot_data__rosidl_typesupport_introspection_c.h"
#include "torsobot_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "torsobot_interfaces/msg/detail/torsobot_data__functions.h"
#include "torsobot_interfaces/msg/detail/torsobot_data__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void torsobot_interfaces__msg__TorsobotData__rosidl_typesupport_introspection_c__TorsobotData_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  torsobot_interfaces__msg__TorsobotData__init(message_memory);
}

void torsobot_interfaces__msg__TorsobotData__rosidl_typesupport_introspection_c__TorsobotData_fini_function(void * message_memory)
{
  torsobot_interfaces__msg__TorsobotData__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember torsobot_interfaces__msg__TorsobotData__rosidl_typesupport_introspection_c__TorsobotData_message_member_array[6] = {
  {
    "torso_pitch",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(torsobot_interfaces__msg__TorsobotData, torso_pitch),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "torso_pitch_rate",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(torsobot_interfaces__msg__TorsobotData, torso_pitch_rate),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "motor_pos",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(torsobot_interfaces__msg__TorsobotData, motor_pos),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "motor_vel",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(torsobot_interfaces__msg__TorsobotData, motor_vel),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "motor_torque",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(torsobot_interfaces__msg__TorsobotData, motor_torque),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "motor_drv_mode",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(torsobot_interfaces__msg__TorsobotData, motor_drv_mode),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers torsobot_interfaces__msg__TorsobotData__rosidl_typesupport_introspection_c__TorsobotData_message_members = {
  "torsobot_interfaces__msg",  // message namespace
  "TorsobotData",  // message name
  6,  // number of fields
  sizeof(torsobot_interfaces__msg__TorsobotData),
  false,  // has_any_key_member_
  torsobot_interfaces__msg__TorsobotData__rosidl_typesupport_introspection_c__TorsobotData_message_member_array,  // message members
  torsobot_interfaces__msg__TorsobotData__rosidl_typesupport_introspection_c__TorsobotData_init_function,  // function to initialize message memory (memory has to be allocated)
  torsobot_interfaces__msg__TorsobotData__rosidl_typesupport_introspection_c__TorsobotData_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t torsobot_interfaces__msg__TorsobotData__rosidl_typesupport_introspection_c__TorsobotData_message_type_support_handle = {
  0,
  &torsobot_interfaces__msg__TorsobotData__rosidl_typesupport_introspection_c__TorsobotData_message_members,
  get_message_typesupport_handle_function,
  &torsobot_interfaces__msg__TorsobotData__get_type_hash,
  &torsobot_interfaces__msg__TorsobotData__get_type_description,
  &torsobot_interfaces__msg__TorsobotData__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_torsobot_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, torsobot_interfaces, msg, TorsobotData)() {
  if (!torsobot_interfaces__msg__TorsobotData__rosidl_typesupport_introspection_c__TorsobotData_message_type_support_handle.typesupport_identifier) {
    torsobot_interfaces__msg__TorsobotData__rosidl_typesupport_introspection_c__TorsobotData_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &torsobot_interfaces__msg__TorsobotData__rosidl_typesupport_introspection_c__TorsobotData_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
