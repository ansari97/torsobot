// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from torsobot_interfaces:srv/RequestControlParams.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "torsobot_interfaces/srv/request_control_params.h"


#ifndef TORSOBOT_INTERFACES__SRV__DETAIL__REQUEST_CONTROL_PARAMS__STRUCT_H_
#define TORSOBOT_INTERFACES__SRV__DETAIL__REQUEST_CONTROL_PARAMS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/RequestControlParams in the package torsobot_interfaces.
typedef struct torsobot_interfaces__srv__RequestControlParams_Request
{
  uint8_t structure_needs_at_least_one_member;
} torsobot_interfaces__srv__RequestControlParams_Request;

// Struct for a sequence of torsobot_interfaces__srv__RequestControlParams_Request.
typedef struct torsobot_interfaces__srv__RequestControlParams_Request__Sequence
{
  torsobot_interfaces__srv__RequestControlParams_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} torsobot_interfaces__srv__RequestControlParams_Request__Sequence;

// Constants defined in the message

/// Struct defined in srv/RequestControlParams in the package torsobot_interfaces.
typedef struct torsobot_interfaces__srv__RequestControlParams_Response
{
  double desired_torso_pitch;
  double mot_max_torque;
  double kp;
  double ki;
  double kd;
} torsobot_interfaces__srv__RequestControlParams_Response;

// Struct for a sequence of torsobot_interfaces__srv__RequestControlParams_Response.
typedef struct torsobot_interfaces__srv__RequestControlParams_Response__Sequence
{
  torsobot_interfaces__srv__RequestControlParams_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} torsobot_interfaces__srv__RequestControlParams_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  torsobot_interfaces__srv__RequestControlParams_Event__request__MAX_SIZE = 1
};
// response
enum
{
  torsobot_interfaces__srv__RequestControlParams_Event__response__MAX_SIZE = 1
};

/// Struct defined in srv/RequestControlParams in the package torsobot_interfaces.
typedef struct torsobot_interfaces__srv__RequestControlParams_Event
{
  service_msgs__msg__ServiceEventInfo info;
  torsobot_interfaces__srv__RequestControlParams_Request__Sequence request;
  torsobot_interfaces__srv__RequestControlParams_Response__Sequence response;
} torsobot_interfaces__srv__RequestControlParams_Event;

// Struct for a sequence of torsobot_interfaces__srv__RequestControlParams_Event.
typedef struct torsobot_interfaces__srv__RequestControlParams_Event__Sequence
{
  torsobot_interfaces__srv__RequestControlParams_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} torsobot_interfaces__srv__RequestControlParams_Event__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // TORSOBOT_INTERFACES__SRV__DETAIL__REQUEST_CONTROL_PARAMS__STRUCT_H_
