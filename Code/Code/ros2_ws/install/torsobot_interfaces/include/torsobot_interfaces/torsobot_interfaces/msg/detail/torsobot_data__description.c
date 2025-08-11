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
      0x3a, 0xc4, 0x2f, 0x7e, 0x10, 0x11, 0x67, 0xd4,
      0x8d, 0x50, 0xa0, 0xd8, 0x2f, 0x08, 0x66, 0x0b,
      0x16, 0xf7, 0x59, 0x7e, 0xd3, 0x68, 0xeb, 0x4e,
      0x5f, 0x01, 0x3c, 0xe1, 0x1b, 0xca, 0x55, 0xcf,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "torsobot_interfaces/msg/detail/torsobot_state__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t torsobot_interfaces__msg__TorsobotState__EXPECTED_HASH = {1, {
    0x55, 0x30, 0xd0, 0x84, 0x7b, 0xfe, 0xfa, 0xaf,
    0xea, 0x2c, 0x1d, 0x16, 0x23, 0x58, 0x64, 0x07,
    0xfa, 0xb7, 0xa4, 0x7d, 0x0c, 0x98, 0x64, 0xee,
    0x03, 0x87, 0xa0, 0x80, 0x36, 0x0d, 0x0d, 0xae,
  }};
#endif

static char torsobot_interfaces__msg__TorsobotData__TYPE_NAME[] = "torsobot_interfaces/msg/TorsobotData";
static char torsobot_interfaces__msg__TorsobotState__TYPE_NAME[] = "torsobot_interfaces/msg/TorsobotState";

// Define type names, field names, and default values
static char torsobot_interfaces__msg__TorsobotData__FIELD_NAME__torsobot_state[] = "torsobot_state";
static char torsobot_interfaces__msg__TorsobotData__FIELD_NAME__wheel_torque[] = "wheel_torque";
static char torsobot_interfaces__msg__TorsobotData__FIELD_NAME__wheel_cmd_torque[] = "wheel_cmd_torque";
static char torsobot_interfaces__msg__TorsobotData__FIELD_NAME__mot_drv_mode[] = "mot_drv_mode";

static rosidl_runtime_c__type_description__Field torsobot_interfaces__msg__TorsobotData__FIELDS[] = {
  {
    {torsobot_interfaces__msg__TorsobotData__FIELD_NAME__torsobot_state, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {torsobot_interfaces__msg__TorsobotState__TYPE_NAME, 37, 37},
    },
    {NULL, 0, 0},
  },
  {
    {torsobot_interfaces__msg__TorsobotData__FIELD_NAME__wheel_torque, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {torsobot_interfaces__msg__TorsobotData__FIELD_NAME__wheel_cmd_torque, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {torsobot_interfaces__msg__TorsobotData__FIELD_NAME__mot_drv_mode, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription torsobot_interfaces__msg__TorsobotData__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {torsobot_interfaces__msg__TorsobotState__TYPE_NAME, 37, 37},
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
      {torsobot_interfaces__msg__TorsobotData__FIELDS, 4, 4},
    },
    {torsobot_interfaces__msg__TorsobotData__REFERENCED_TYPE_DESCRIPTIONS, 1, 1},
  };
  if (!constructed) {
    assert(0 == memcmp(&torsobot_interfaces__msg__TorsobotState__EXPECTED_HASH, torsobot_interfaces__msg__TorsobotState__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = torsobot_interfaces__msg__TorsobotState__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "TorsobotState torsobot_state\n"
  "float64 wheel_torque\n"
  "float64 wheel_cmd_torque\n"
  "int8 mot_drv_mode";

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
    {toplevel_type_raw_source, 92, 92},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
torsobot_interfaces__msg__TorsobotData__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[2];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 2, 2};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *torsobot_interfaces__msg__TorsobotData__get_individual_type_description_source(NULL),
    sources[1] = *torsobot_interfaces__msg__TorsobotState__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
