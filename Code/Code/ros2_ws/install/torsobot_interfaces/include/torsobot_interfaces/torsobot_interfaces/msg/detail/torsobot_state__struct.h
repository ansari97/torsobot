// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from torsobot_interfaces:msg/TorsobotState.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "torsobot_interfaces/msg/torsobot_state.h"


#ifndef TORSOBOT_INTERFACES__MSG__DETAIL__TORSOBOT_STATE__STRUCT_H_
#define TORSOBOT_INTERFACES__MSG__DETAIL__TORSOBOT_STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

/// Struct defined in msg/TorsobotState in the package torsobot_interfaces.
typedef struct torsobot_interfaces__msg__TorsobotState
{
  double torso_pitch;
  double torso_pitch_rate;
  double wheel_pos;
  double wheel_vel;
} torsobot_interfaces__msg__TorsobotState;

// Struct for a sequence of torsobot_interfaces__msg__TorsobotState.
typedef struct torsobot_interfaces__msg__TorsobotState__Sequence
{
  torsobot_interfaces__msg__TorsobotState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} torsobot_interfaces__msg__TorsobotState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // TORSOBOT_INTERFACES__MSG__DETAIL__TORSOBOT_STATE__STRUCT_H_
