// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from torsobot_interfaces:srv/RequestControlParams.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "torsobot_interfaces/srv/detail/request_control_params__rosidl_typesupport_introspection_c.h"
#include "torsobot_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "torsobot_interfaces/srv/detail/request_control_params__functions.h"
#include "torsobot_interfaces/srv/detail/request_control_params__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void torsobot_interfaces__srv__RequestControlParams_Request__rosidl_typesupport_introspection_c__RequestControlParams_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  torsobot_interfaces__srv__RequestControlParams_Request__init(message_memory);
}

void torsobot_interfaces__srv__RequestControlParams_Request__rosidl_typesupport_introspection_c__RequestControlParams_Request_fini_function(void * message_memory)
{
  torsobot_interfaces__srv__RequestControlParams_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember torsobot_interfaces__srv__RequestControlParams_Request__rosidl_typesupport_introspection_c__RequestControlParams_Request_message_member_array[1] = {
  {
    "structure_needs_at_least_one_member",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(torsobot_interfaces__srv__RequestControlParams_Request, structure_needs_at_least_one_member),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers torsobot_interfaces__srv__RequestControlParams_Request__rosidl_typesupport_introspection_c__RequestControlParams_Request_message_members = {
  "torsobot_interfaces__srv",  // message namespace
  "RequestControlParams_Request",  // message name
  1,  // number of fields
  sizeof(torsobot_interfaces__srv__RequestControlParams_Request),
  false,  // has_any_key_member_
  torsobot_interfaces__srv__RequestControlParams_Request__rosidl_typesupport_introspection_c__RequestControlParams_Request_message_member_array,  // message members
  torsobot_interfaces__srv__RequestControlParams_Request__rosidl_typesupport_introspection_c__RequestControlParams_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  torsobot_interfaces__srv__RequestControlParams_Request__rosidl_typesupport_introspection_c__RequestControlParams_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t torsobot_interfaces__srv__RequestControlParams_Request__rosidl_typesupport_introspection_c__RequestControlParams_Request_message_type_support_handle = {
  0,
  &torsobot_interfaces__srv__RequestControlParams_Request__rosidl_typesupport_introspection_c__RequestControlParams_Request_message_members,
  get_message_typesupport_handle_function,
  &torsobot_interfaces__srv__RequestControlParams_Request__get_type_hash,
  &torsobot_interfaces__srv__RequestControlParams_Request__get_type_description,
  &torsobot_interfaces__srv__RequestControlParams_Request__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_torsobot_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, torsobot_interfaces, srv, RequestControlParams_Request)() {
  if (!torsobot_interfaces__srv__RequestControlParams_Request__rosidl_typesupport_introspection_c__RequestControlParams_Request_message_type_support_handle.typesupport_identifier) {
    torsobot_interfaces__srv__RequestControlParams_Request__rosidl_typesupport_introspection_c__RequestControlParams_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &torsobot_interfaces__srv__RequestControlParams_Request__rosidl_typesupport_introspection_c__RequestControlParams_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "torsobot_interfaces/srv/detail/request_control_params__rosidl_typesupport_introspection_c.h"
// already included above
// #include "torsobot_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "torsobot_interfaces/srv/detail/request_control_params__functions.h"
// already included above
// #include "torsobot_interfaces/srv/detail/request_control_params__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void torsobot_interfaces__srv__RequestControlParams_Response__rosidl_typesupport_introspection_c__RequestControlParams_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  torsobot_interfaces__srv__RequestControlParams_Response__init(message_memory);
}

void torsobot_interfaces__srv__RequestControlParams_Response__rosidl_typesupport_introspection_c__RequestControlParams_Response_fini_function(void * message_memory)
{
  torsobot_interfaces__srv__RequestControlParams_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember torsobot_interfaces__srv__RequestControlParams_Response__rosidl_typesupport_introspection_c__RequestControlParams_Response_message_member_array[5] = {
  {
    "desired_torso_pitch",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(torsobot_interfaces__srv__RequestControlParams_Response, desired_torso_pitch),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "mot_max_torque",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(torsobot_interfaces__srv__RequestControlParams_Response, mot_max_torque),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "kp",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(torsobot_interfaces__srv__RequestControlParams_Response, kp),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "ki",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(torsobot_interfaces__srv__RequestControlParams_Response, ki),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "kd",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(torsobot_interfaces__srv__RequestControlParams_Response, kd),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers torsobot_interfaces__srv__RequestControlParams_Response__rosidl_typesupport_introspection_c__RequestControlParams_Response_message_members = {
  "torsobot_interfaces__srv",  // message namespace
  "RequestControlParams_Response",  // message name
  5,  // number of fields
  sizeof(torsobot_interfaces__srv__RequestControlParams_Response),
  false,  // has_any_key_member_
  torsobot_interfaces__srv__RequestControlParams_Response__rosidl_typesupport_introspection_c__RequestControlParams_Response_message_member_array,  // message members
  torsobot_interfaces__srv__RequestControlParams_Response__rosidl_typesupport_introspection_c__RequestControlParams_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  torsobot_interfaces__srv__RequestControlParams_Response__rosidl_typesupport_introspection_c__RequestControlParams_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t torsobot_interfaces__srv__RequestControlParams_Response__rosidl_typesupport_introspection_c__RequestControlParams_Response_message_type_support_handle = {
  0,
  &torsobot_interfaces__srv__RequestControlParams_Response__rosidl_typesupport_introspection_c__RequestControlParams_Response_message_members,
  get_message_typesupport_handle_function,
  &torsobot_interfaces__srv__RequestControlParams_Response__get_type_hash,
  &torsobot_interfaces__srv__RequestControlParams_Response__get_type_description,
  &torsobot_interfaces__srv__RequestControlParams_Response__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_torsobot_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, torsobot_interfaces, srv, RequestControlParams_Response)() {
  if (!torsobot_interfaces__srv__RequestControlParams_Response__rosidl_typesupport_introspection_c__RequestControlParams_Response_message_type_support_handle.typesupport_identifier) {
    torsobot_interfaces__srv__RequestControlParams_Response__rosidl_typesupport_introspection_c__RequestControlParams_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &torsobot_interfaces__srv__RequestControlParams_Response__rosidl_typesupport_introspection_c__RequestControlParams_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "torsobot_interfaces/srv/detail/request_control_params__rosidl_typesupport_introspection_c.h"
// already included above
// #include "torsobot_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "torsobot_interfaces/srv/detail/request_control_params__functions.h"
// already included above
// #include "torsobot_interfaces/srv/detail/request_control_params__struct.h"


// Include directives for member types
// Member `info`
#include "service_msgs/msg/service_event_info.h"
// Member `info`
#include "service_msgs/msg/detail/service_event_info__rosidl_typesupport_introspection_c.h"
// Member `request`
// Member `response`
#include "torsobot_interfaces/srv/request_control_params.h"
// Member `request`
// Member `response`
// already included above
// #include "torsobot_interfaces/srv/detail/request_control_params__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void torsobot_interfaces__srv__RequestControlParams_Event__rosidl_typesupport_introspection_c__RequestControlParams_Event_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  torsobot_interfaces__srv__RequestControlParams_Event__init(message_memory);
}

void torsobot_interfaces__srv__RequestControlParams_Event__rosidl_typesupport_introspection_c__RequestControlParams_Event_fini_function(void * message_memory)
{
  torsobot_interfaces__srv__RequestControlParams_Event__fini(message_memory);
}

size_t torsobot_interfaces__srv__RequestControlParams_Event__rosidl_typesupport_introspection_c__size_function__RequestControlParams_Event__request(
  const void * untyped_member)
{
  const torsobot_interfaces__srv__RequestControlParams_Request__Sequence * member =
    (const torsobot_interfaces__srv__RequestControlParams_Request__Sequence *)(untyped_member);
  return member->size;
}

const void * torsobot_interfaces__srv__RequestControlParams_Event__rosidl_typesupport_introspection_c__get_const_function__RequestControlParams_Event__request(
  const void * untyped_member, size_t index)
{
  const torsobot_interfaces__srv__RequestControlParams_Request__Sequence * member =
    (const torsobot_interfaces__srv__RequestControlParams_Request__Sequence *)(untyped_member);
  return &member->data[index];
}

void * torsobot_interfaces__srv__RequestControlParams_Event__rosidl_typesupport_introspection_c__get_function__RequestControlParams_Event__request(
  void * untyped_member, size_t index)
{
  torsobot_interfaces__srv__RequestControlParams_Request__Sequence * member =
    (torsobot_interfaces__srv__RequestControlParams_Request__Sequence *)(untyped_member);
  return &member->data[index];
}

void torsobot_interfaces__srv__RequestControlParams_Event__rosidl_typesupport_introspection_c__fetch_function__RequestControlParams_Event__request(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const torsobot_interfaces__srv__RequestControlParams_Request * item =
    ((const torsobot_interfaces__srv__RequestControlParams_Request *)
    torsobot_interfaces__srv__RequestControlParams_Event__rosidl_typesupport_introspection_c__get_const_function__RequestControlParams_Event__request(untyped_member, index));
  torsobot_interfaces__srv__RequestControlParams_Request * value =
    (torsobot_interfaces__srv__RequestControlParams_Request *)(untyped_value);
  *value = *item;
}

void torsobot_interfaces__srv__RequestControlParams_Event__rosidl_typesupport_introspection_c__assign_function__RequestControlParams_Event__request(
  void * untyped_member, size_t index, const void * untyped_value)
{
  torsobot_interfaces__srv__RequestControlParams_Request * item =
    ((torsobot_interfaces__srv__RequestControlParams_Request *)
    torsobot_interfaces__srv__RequestControlParams_Event__rosidl_typesupport_introspection_c__get_function__RequestControlParams_Event__request(untyped_member, index));
  const torsobot_interfaces__srv__RequestControlParams_Request * value =
    (const torsobot_interfaces__srv__RequestControlParams_Request *)(untyped_value);
  *item = *value;
}

bool torsobot_interfaces__srv__RequestControlParams_Event__rosidl_typesupport_introspection_c__resize_function__RequestControlParams_Event__request(
  void * untyped_member, size_t size)
{
  torsobot_interfaces__srv__RequestControlParams_Request__Sequence * member =
    (torsobot_interfaces__srv__RequestControlParams_Request__Sequence *)(untyped_member);
  torsobot_interfaces__srv__RequestControlParams_Request__Sequence__fini(member);
  return torsobot_interfaces__srv__RequestControlParams_Request__Sequence__init(member, size);
}

size_t torsobot_interfaces__srv__RequestControlParams_Event__rosidl_typesupport_introspection_c__size_function__RequestControlParams_Event__response(
  const void * untyped_member)
{
  const torsobot_interfaces__srv__RequestControlParams_Response__Sequence * member =
    (const torsobot_interfaces__srv__RequestControlParams_Response__Sequence *)(untyped_member);
  return member->size;
}

const void * torsobot_interfaces__srv__RequestControlParams_Event__rosidl_typesupport_introspection_c__get_const_function__RequestControlParams_Event__response(
  const void * untyped_member, size_t index)
{
  const torsobot_interfaces__srv__RequestControlParams_Response__Sequence * member =
    (const torsobot_interfaces__srv__RequestControlParams_Response__Sequence *)(untyped_member);
  return &member->data[index];
}

void * torsobot_interfaces__srv__RequestControlParams_Event__rosidl_typesupport_introspection_c__get_function__RequestControlParams_Event__response(
  void * untyped_member, size_t index)
{
  torsobot_interfaces__srv__RequestControlParams_Response__Sequence * member =
    (torsobot_interfaces__srv__RequestControlParams_Response__Sequence *)(untyped_member);
  return &member->data[index];
}

void torsobot_interfaces__srv__RequestControlParams_Event__rosidl_typesupport_introspection_c__fetch_function__RequestControlParams_Event__response(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const torsobot_interfaces__srv__RequestControlParams_Response * item =
    ((const torsobot_interfaces__srv__RequestControlParams_Response *)
    torsobot_interfaces__srv__RequestControlParams_Event__rosidl_typesupport_introspection_c__get_const_function__RequestControlParams_Event__response(untyped_member, index));
  torsobot_interfaces__srv__RequestControlParams_Response * value =
    (torsobot_interfaces__srv__RequestControlParams_Response *)(untyped_value);
  *value = *item;
}

void torsobot_interfaces__srv__RequestControlParams_Event__rosidl_typesupport_introspection_c__assign_function__RequestControlParams_Event__response(
  void * untyped_member, size_t index, const void * untyped_value)
{
  torsobot_interfaces__srv__RequestControlParams_Response * item =
    ((torsobot_interfaces__srv__RequestControlParams_Response *)
    torsobot_interfaces__srv__RequestControlParams_Event__rosidl_typesupport_introspection_c__get_function__RequestControlParams_Event__response(untyped_member, index));
  const torsobot_interfaces__srv__RequestControlParams_Response * value =
    (const torsobot_interfaces__srv__RequestControlParams_Response *)(untyped_value);
  *item = *value;
}

bool torsobot_interfaces__srv__RequestControlParams_Event__rosidl_typesupport_introspection_c__resize_function__RequestControlParams_Event__response(
  void * untyped_member, size_t size)
{
  torsobot_interfaces__srv__RequestControlParams_Response__Sequence * member =
    (torsobot_interfaces__srv__RequestControlParams_Response__Sequence *)(untyped_member);
  torsobot_interfaces__srv__RequestControlParams_Response__Sequence__fini(member);
  return torsobot_interfaces__srv__RequestControlParams_Response__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember torsobot_interfaces__srv__RequestControlParams_Event__rosidl_typesupport_introspection_c__RequestControlParams_Event_message_member_array[3] = {
  {
    "info",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(torsobot_interfaces__srv__RequestControlParams_Event, info),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "request",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    true,  // is array
    1,  // array size
    true,  // is upper bound
    offsetof(torsobot_interfaces__srv__RequestControlParams_Event, request),  // bytes offset in struct
    NULL,  // default value
    torsobot_interfaces__srv__RequestControlParams_Event__rosidl_typesupport_introspection_c__size_function__RequestControlParams_Event__request,  // size() function pointer
    torsobot_interfaces__srv__RequestControlParams_Event__rosidl_typesupport_introspection_c__get_const_function__RequestControlParams_Event__request,  // get_const(index) function pointer
    torsobot_interfaces__srv__RequestControlParams_Event__rosidl_typesupport_introspection_c__get_function__RequestControlParams_Event__request,  // get(index) function pointer
    torsobot_interfaces__srv__RequestControlParams_Event__rosidl_typesupport_introspection_c__fetch_function__RequestControlParams_Event__request,  // fetch(index, &value) function pointer
    torsobot_interfaces__srv__RequestControlParams_Event__rosidl_typesupport_introspection_c__assign_function__RequestControlParams_Event__request,  // assign(index, value) function pointer
    torsobot_interfaces__srv__RequestControlParams_Event__rosidl_typesupport_introspection_c__resize_function__RequestControlParams_Event__request  // resize(index) function pointer
  },
  {
    "response",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    true,  // is array
    1,  // array size
    true,  // is upper bound
    offsetof(torsobot_interfaces__srv__RequestControlParams_Event, response),  // bytes offset in struct
    NULL,  // default value
    torsobot_interfaces__srv__RequestControlParams_Event__rosidl_typesupport_introspection_c__size_function__RequestControlParams_Event__response,  // size() function pointer
    torsobot_interfaces__srv__RequestControlParams_Event__rosidl_typesupport_introspection_c__get_const_function__RequestControlParams_Event__response,  // get_const(index) function pointer
    torsobot_interfaces__srv__RequestControlParams_Event__rosidl_typesupport_introspection_c__get_function__RequestControlParams_Event__response,  // get(index) function pointer
    torsobot_interfaces__srv__RequestControlParams_Event__rosidl_typesupport_introspection_c__fetch_function__RequestControlParams_Event__response,  // fetch(index, &value) function pointer
    torsobot_interfaces__srv__RequestControlParams_Event__rosidl_typesupport_introspection_c__assign_function__RequestControlParams_Event__response,  // assign(index, value) function pointer
    torsobot_interfaces__srv__RequestControlParams_Event__rosidl_typesupport_introspection_c__resize_function__RequestControlParams_Event__response  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers torsobot_interfaces__srv__RequestControlParams_Event__rosidl_typesupport_introspection_c__RequestControlParams_Event_message_members = {
  "torsobot_interfaces__srv",  // message namespace
  "RequestControlParams_Event",  // message name
  3,  // number of fields
  sizeof(torsobot_interfaces__srv__RequestControlParams_Event),
  false,  // has_any_key_member_
  torsobot_interfaces__srv__RequestControlParams_Event__rosidl_typesupport_introspection_c__RequestControlParams_Event_message_member_array,  // message members
  torsobot_interfaces__srv__RequestControlParams_Event__rosidl_typesupport_introspection_c__RequestControlParams_Event_init_function,  // function to initialize message memory (memory has to be allocated)
  torsobot_interfaces__srv__RequestControlParams_Event__rosidl_typesupport_introspection_c__RequestControlParams_Event_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t torsobot_interfaces__srv__RequestControlParams_Event__rosidl_typesupport_introspection_c__RequestControlParams_Event_message_type_support_handle = {
  0,
  &torsobot_interfaces__srv__RequestControlParams_Event__rosidl_typesupport_introspection_c__RequestControlParams_Event_message_members,
  get_message_typesupport_handle_function,
  &torsobot_interfaces__srv__RequestControlParams_Event__get_type_hash,
  &torsobot_interfaces__srv__RequestControlParams_Event__get_type_description,
  &torsobot_interfaces__srv__RequestControlParams_Event__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_torsobot_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, torsobot_interfaces, srv, RequestControlParams_Event)() {
  torsobot_interfaces__srv__RequestControlParams_Event__rosidl_typesupport_introspection_c__RequestControlParams_Event_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, service_msgs, msg, ServiceEventInfo)();
  torsobot_interfaces__srv__RequestControlParams_Event__rosidl_typesupport_introspection_c__RequestControlParams_Event_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, torsobot_interfaces, srv, RequestControlParams_Request)();
  torsobot_interfaces__srv__RequestControlParams_Event__rosidl_typesupport_introspection_c__RequestControlParams_Event_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, torsobot_interfaces, srv, RequestControlParams_Response)();
  if (!torsobot_interfaces__srv__RequestControlParams_Event__rosidl_typesupport_introspection_c__RequestControlParams_Event_message_type_support_handle.typesupport_identifier) {
    torsobot_interfaces__srv__RequestControlParams_Event__rosidl_typesupport_introspection_c__RequestControlParams_Event_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &torsobot_interfaces__srv__RequestControlParams_Event__rosidl_typesupport_introspection_c__RequestControlParams_Event_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "torsobot_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "torsobot_interfaces/srv/detail/request_control_params__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers torsobot_interfaces__srv__detail__request_control_params__rosidl_typesupport_introspection_c__RequestControlParams_service_members = {
  "torsobot_interfaces__srv",  // service namespace
  "RequestControlParams",  // service name
  // the following fields are initialized below on first access
  NULL,  // request message
  // torsobot_interfaces__srv__detail__request_control_params__rosidl_typesupport_introspection_c__RequestControlParams_Request_message_type_support_handle,
  NULL,  // response message
  // torsobot_interfaces__srv__detail__request_control_params__rosidl_typesupport_introspection_c__RequestControlParams_Response_message_type_support_handle
  NULL  // event_message
  // torsobot_interfaces__srv__detail__request_control_params__rosidl_typesupport_introspection_c__RequestControlParams_Response_message_type_support_handle
};


static rosidl_service_type_support_t torsobot_interfaces__srv__detail__request_control_params__rosidl_typesupport_introspection_c__RequestControlParams_service_type_support_handle = {
  0,
  &torsobot_interfaces__srv__detail__request_control_params__rosidl_typesupport_introspection_c__RequestControlParams_service_members,
  get_service_typesupport_handle_function,
  &torsobot_interfaces__srv__RequestControlParams_Request__rosidl_typesupport_introspection_c__RequestControlParams_Request_message_type_support_handle,
  &torsobot_interfaces__srv__RequestControlParams_Response__rosidl_typesupport_introspection_c__RequestControlParams_Response_message_type_support_handle,
  &torsobot_interfaces__srv__RequestControlParams_Event__rosidl_typesupport_introspection_c__RequestControlParams_Event_message_type_support_handle,
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_CREATE_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    torsobot_interfaces,
    srv,
    RequestControlParams
  ),
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_DESTROY_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    torsobot_interfaces,
    srv,
    RequestControlParams
  ),
  &torsobot_interfaces__srv__RequestControlParams__get_type_hash,
  &torsobot_interfaces__srv__RequestControlParams__get_type_description,
  &torsobot_interfaces__srv__RequestControlParams__get_type_description_sources,
};

// Forward declaration of message type support functions for service members
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, torsobot_interfaces, srv, RequestControlParams_Request)(void);

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, torsobot_interfaces, srv, RequestControlParams_Response)(void);

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, torsobot_interfaces, srv, RequestControlParams_Event)(void);

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_torsobot_interfaces
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, torsobot_interfaces, srv, RequestControlParams)(void) {
  if (!torsobot_interfaces__srv__detail__request_control_params__rosidl_typesupport_introspection_c__RequestControlParams_service_type_support_handle.typesupport_identifier) {
    torsobot_interfaces__srv__detail__request_control_params__rosidl_typesupport_introspection_c__RequestControlParams_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)torsobot_interfaces__srv__detail__request_control_params__rosidl_typesupport_introspection_c__RequestControlParams_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, torsobot_interfaces, srv, RequestControlParams_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, torsobot_interfaces, srv, RequestControlParams_Response)()->data;
  }
  if (!service_members->event_members_) {
    service_members->event_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, torsobot_interfaces, srv, RequestControlParams_Event)()->data;
  }

  return &torsobot_interfaces__srv__detail__request_control_params__rosidl_typesupport_introspection_c__RequestControlParams_service_type_support_handle;
}
