// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from torsobot_interfaces:msg/TorsobotData.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "torsobot_interfaces/msg/torsobot_data.h"


#ifndef TORSOBOT_INTERFACES__MSG__DETAIL__TORSOBOT_DATA__STRUCT_H_
#define TORSOBOT_INTERFACES__MSG__DETAIL__TORSOBOT_DATA__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

/// Struct defined in msg/TorsobotData in the package torsobot_interfaces.
typedef struct torsobot_interfaces__msg__TorsobotData
{
  double torso_pitch;
  double torso_pitch_rate;
  double motor_pos;
  double motor_vel;
  double motor_torque;
  int8_t motor_drv_mode;
} torsobot_interfaces__msg__TorsobotData;

// Struct for a sequence of torsobot_interfaces__msg__TorsobotData.
typedef struct torsobot_interfaces__msg__TorsobotData__Sequence
{
  torsobot_interfaces__msg__TorsobotData * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} torsobot_interfaces__msg__TorsobotData__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // TORSOBOT_INTERFACES__MSG__DETAIL__TORSOBOT_DATA__STRUCT_H_
