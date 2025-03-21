// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from torsobot_interfaces:msg/Data.idl
// generated code does not contain a copyright notice

#include "torsobot_interfaces/msg/detail/data__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_torsobot_interfaces
const rosidl_type_hash_t *
torsobot_interfaces__msg__Data__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x1f, 0x92, 0xca, 0xc2, 0x83, 0x14, 0x84, 0x05,
      0x14, 0xfb, 0xee, 0xa9, 0x77, 0x08, 0x67, 0x5c,
      0xb9, 0xa3, 0x62, 0x25, 0xa0, 0x0c, 0x42, 0xdb,
      0x7f, 0x54, 0xd5, 0x82, 0xa3, 0x1e, 0xdb, 0x08,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char torsobot_interfaces__msg__Data__TYPE_NAME[] = "torsobot_interfaces/msg/Data";

// Define type names, field names, and default values
static char torsobot_interfaces__msg__Data__FIELD_NAME__torso_pitch[] = "torso_pitch";
static char torsobot_interfaces__msg__Data__FIELD_NAME__wheel_pitch[] = "wheel_pitch";
static char torsobot_interfaces__msg__Data__FIELD_NAME__motor_current[] = "motor_current";

static rosidl_runtime_c__type_description__Field torsobot_interfaces__msg__Data__FIELDS[] = {
  {
    {torsobot_interfaces__msg__Data__FIELD_NAME__torso_pitch, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {torsobot_interfaces__msg__Data__FIELD_NAME__wheel_pitch, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {torsobot_interfaces__msg__Data__FIELD_NAME__motor_current, 13, 13},
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
torsobot_interfaces__msg__Data__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {torsobot_interfaces__msg__Data__TYPE_NAME, 28, 28},
      {torsobot_interfaces__msg__Data__FIELDS, 3, 3},
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
  "float64 wheel_pitch\n"
  "float64 motor_current";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
torsobot_interfaces__msg__Data__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {torsobot_interfaces__msg__Data__TYPE_NAME, 28, 28},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 61, 61},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
torsobot_interfaces__msg__Data__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *torsobot_interfaces__msg__Data__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
