// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from torsobot_interfaces:msg/TorsobotState.idl
// generated code does not contain a copyright notice

#include "torsobot_interfaces/msg/detail/torsobot_state__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_torsobot_interfaces
const rosidl_type_hash_t *
torsobot_interfaces__msg__TorsobotState__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x55, 0x30, 0xd0, 0x84, 0x7b, 0xfe, 0xfa, 0xaf,
      0xea, 0x2c, 0x1d, 0x16, 0x23, 0x58, 0x64, 0x07,
      0xfa, 0xb7, 0xa4, 0x7d, 0x0c, 0x98, 0x64, 0xee,
      0x03, 0x87, 0xa0, 0x80, 0x36, 0x0d, 0x0d, 0xae,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char torsobot_interfaces__msg__TorsobotState__TYPE_NAME[] = "torsobot_interfaces/msg/TorsobotState";

// Define type names, field names, and default values
static char torsobot_interfaces__msg__TorsobotState__FIELD_NAME__torso_pitch[] = "torso_pitch";
static char torsobot_interfaces__msg__TorsobotState__FIELD_NAME__torso_pitch_rate[] = "torso_pitch_rate";
static char torsobot_interfaces__msg__TorsobotState__FIELD_NAME__wheel_pos[] = "wheel_pos";
static char torsobot_interfaces__msg__TorsobotState__FIELD_NAME__wheel_vel[] = "wheel_vel";

static rosidl_runtime_c__type_description__Field torsobot_interfaces__msg__TorsobotState__FIELDS[] = {
  {
    {torsobot_interfaces__msg__TorsobotState__FIELD_NAME__torso_pitch, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {torsobot_interfaces__msg__TorsobotState__FIELD_NAME__torso_pitch_rate, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {torsobot_interfaces__msg__TorsobotState__FIELD_NAME__wheel_pos, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {torsobot_interfaces__msg__TorsobotState__FIELD_NAME__wheel_vel, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
torsobot_interfaces__msg__TorsobotState__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {torsobot_interfaces__msg__TorsobotState__TYPE_NAME, 37, 37},
      {torsobot_interfaces__msg__TorsobotState__FIELDS, 4, 4},
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
  "float64 wheel_pos\n"
  "float64 wheel_vel";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
torsobot_interfaces__msg__TorsobotState__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {torsobot_interfaces__msg__TorsobotState__TYPE_NAME, 37, 37},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 80, 80},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
torsobot_interfaces__msg__TorsobotState__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *torsobot_interfaces__msg__TorsobotState__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
