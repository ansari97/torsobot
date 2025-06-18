// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from torsobot_interfaces:msg/TorsobotData.idl
// generated code does not contain a copyright notice

#include "torsobot_interfaces/msg/detail/torsobot_data__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_torsobot_interfaces
const rosidl_type_hash_t *
torsobot_interfaces__msg__TorsobotData__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x81, 0x67, 0x41, 0xea, 0x2c, 0xca, 0xa9, 0x1a,
      0xa0, 0xe1, 0x58, 0xda, 0x62, 0x15, 0x36, 0xed,
      0x06, 0x5e, 0xb0, 0xe5, 0x12, 0x28, 0xfe, 0x3c,
      0x72, 0xd0, 0x71, 0x4a, 0xfc, 0x1b, 0x85, 0xcb,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char torsobot_interfaces__msg__TorsobotData__TYPE_NAME[] = "torsobot_interfaces/msg/TorsobotData";

// Define type names, field names, and default values
static char torsobot_interfaces__msg__TorsobotData__FIELD_NAME__torso_pitch[] = "torso_pitch";
static char torsobot_interfaces__msg__TorsobotData__FIELD_NAME__torso_pitch_rate[] = "torso_pitch_rate";
static char torsobot_interfaces__msg__TorsobotData__FIELD_NAME__motor_pos[] = "motor_pos";
static char torsobot_interfaces__msg__TorsobotData__FIELD_NAME__motor_vel[] = "motor_vel";
static char torsobot_interfaces__msg__TorsobotData__FIELD_NAME__motor_torque[] = "motor_torque";
static char torsobot_interfaces__msg__TorsobotData__FIELD_NAME__motor_drv_mode[] = "motor_drv_mode";

static rosidl_runtime_c__type_description__Field torsobot_interfaces__msg__TorsobotData__FIELDS[] = {
  {
    {torsobot_interfaces__msg__TorsobotData__FIELD_NAME__torso_pitch, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {torsobot_interfaces__msg__TorsobotData__FIELD_NAME__torso_pitch_rate, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {torsobot_interfaces__msg__TorsobotData__FIELD_NAME__motor_pos, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {torsobot_interfaces__msg__TorsobotData__FIELD_NAME__motor_vel, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {torsobot_interfaces__msg__TorsobotData__FIELD_NAME__motor_torque, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {torsobot_interfaces__msg__TorsobotData__FIELD_NAME__motor_drv_mode, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
torsobot_interfaces__msg__TorsobotData__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {torsobot_interfaces__msg__TorsobotData__TYPE_NAME, 36, 36},
      {torsobot_interfaces__msg__TorsobotData__FIELDS, 6, 6},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "float64 torso_pitch\n"
  "float64 torso_pitch_rate\n"
  "float64 motor_pos\n"
  "float64 motor_vel\n"
  "float64 motor_torque\n"
  "int8 motor_drv_mode";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
torsobot_interfaces__msg__TorsobotData__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {torsobot_interfaces__msg__TorsobotData__TYPE_NAME, 36, 36},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 121, 121},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
torsobot_interfaces__msg__TorsobotData__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *torsobot_interfaces__msg__TorsobotData__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
