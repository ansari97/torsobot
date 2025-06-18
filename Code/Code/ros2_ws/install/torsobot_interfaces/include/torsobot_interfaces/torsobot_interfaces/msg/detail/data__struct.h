// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from torsobot_interfaces:msg/Data.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "torsobot_interfaces/msg/data.h"


#ifndef TORSOBOT_INTERFACES__MSG__DETAIL__DATA__STRUCT_H_
#define TORSOBOT_INTERFACES__MSG__DETAIL__DATA__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

/// Struct defined in msg/Data in the package torsobot_interfaces.
typedef struct torsobot_interfaces__msg__Data
{
  double torso_pitch;
  double wheel_pitch;
  double motor_current;
  double motor_pos;
  double motor_vel;
  double motor_acc;
} torsobot_interfaces__msg__Data;

// Struct for a sequence of torsobot_interfaces__msg__Data.
typedef struct torsobot_interfaces__msg__Data__Sequence
{
  torsobot_interfaces__msg__Data * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} torsobot_interfaces__msg__Data__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // TORSOBOT_INTERFACES__MSG__DETAIL__DATA__STRUCT_H_
