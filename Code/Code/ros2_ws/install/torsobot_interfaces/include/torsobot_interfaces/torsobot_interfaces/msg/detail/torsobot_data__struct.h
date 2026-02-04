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

// Include directives for member types
// Member 'torsobot_state'
#include "torsobot_interfaces/msg/detail/torsobot_state__struct.h"

/// Struct defined in msg/TorsobotData in the package torsobot_interfaces.
typedef struct torsobot_interfaces__msg__TorsobotData
{
  torsobot_interfaces__msg__TorsobotState torsobot_state;
  double wheel_torque;
  double wheel_cmd_torque;
  int8_t mot_drv_mode;
  /// float64 mot_pos
  /// float64 mot_vel
  /// float64 mot_pos_init
  double torso_pitch_init;
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
