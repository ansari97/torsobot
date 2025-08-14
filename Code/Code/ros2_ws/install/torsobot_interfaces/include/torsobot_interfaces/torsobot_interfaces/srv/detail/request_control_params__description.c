// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from torsobot_interfaces:srv/RequestControlParams.idl
// generated code does not contain a copyright notice

#include "torsobot_interfaces/srv/detail/request_control_params__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_torsobot_interfaces
const rosidl_type_hash_t *
torsobot_interfaces__srv__RequestControlParams__get_type_hash(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xa6, 0xef, 0x8d, 0x00, 0xb2, 0x22, 0x7f, 0x86,
      0xdd, 0x81, 0x88, 0xeb, 0xab, 0x0f, 0xd3, 0x6d,
      0xd8, 0x94, 0xf2, 0xd2, 0xbb, 0x7b, 0x03, 0x11,
      0x01, 0x91, 0x54, 0xfd, 0x38, 0x22, 0x9c, 0xe3,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_torsobot_interfaces
const rosidl_type_hash_t *
torsobot_interfaces__srv__RequestControlParams_Request__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x7f, 0x16, 0x17, 0xb1, 0xb6, 0xab, 0x7b, 0xa7,
      0x31, 0x81, 0x6f, 0x17, 0x6d, 0xd9, 0x73, 0xf2,
      0xf1, 0x2b, 0xaa, 0x23, 0x69, 0xf3, 0xe7, 0x43,
      0x0c, 0xa8, 0xe9, 0x98, 0xd0, 0x3d, 0xae, 0xbd,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_torsobot_interfaces
const rosidl_type_hash_t *
torsobot_interfaces__srv__RequestControlParams_Response__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xf0, 0x4b, 0x2d, 0x95, 0x9b, 0x12, 0xa4, 0xcf,
      0xd1, 0x79, 0xda, 0xd8, 0xf9, 0xcd, 0x8f, 0x94,
      0xfd, 0x4e, 0x8a, 0x20, 0x5d, 0x6b, 0xf2, 0x7b,
      0x2e, 0x22, 0x6a, 0x87, 0x31, 0x42, 0xe0, 0xca,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_torsobot_interfaces
const rosidl_type_hash_t *
torsobot_interfaces__srv__RequestControlParams_Event__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x01, 0x7c, 0x8c, 0xd0, 0x14, 0xd8, 0xae, 0x86,
      0x47, 0x34, 0x71, 0x24, 0xf8, 0xca, 0xa4, 0xe4,
      0x45, 0x87, 0xaa, 0x79, 0xef, 0x0c, 0xd8, 0xe6,
      0xd3, 0x1f, 0x40, 0x40, 0x1b, 0xad, 0x32, 0x16,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "builtin_interfaces/msg/detail/time__functions.h"
#include "service_msgs/msg/detail/service_event_info__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t builtin_interfaces__msg__Time__EXPECTED_HASH = {1, {
    0xb1, 0x06, 0x23, 0x5e, 0x25, 0xa4, 0xc5, 0xed,
    0x35, 0x09, 0x8a, 0xa0, 0xa6, 0x1a, 0x3e, 0xe9,
    0xc9, 0xb1, 0x8d, 0x19, 0x7f, 0x39, 0x8b, 0x0e,
    0x42, 0x06, 0xce, 0xa9, 0xac, 0xf9, 0xc1, 0x97,
  }};
static const rosidl_type_hash_t service_msgs__msg__ServiceEventInfo__EXPECTED_HASH = {1, {
    0x41, 0xbc, 0xbb, 0xe0, 0x7a, 0x75, 0xc9, 0xb5,
    0x2b, 0xc9, 0x6b, 0xfd, 0x5c, 0x24, 0xd7, 0xf0,
    0xfc, 0x0a, 0x08, 0xc0, 0xcb, 0x79, 0x21, 0xb3,
    0x37, 0x3c, 0x57, 0x32, 0x34, 0x5a, 0x6f, 0x45,
  }};
#endif

static char torsobot_interfaces__srv__RequestControlParams__TYPE_NAME[] = "torsobot_interfaces/srv/RequestControlParams";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char service_msgs__msg__ServiceEventInfo__TYPE_NAME[] = "service_msgs/msg/ServiceEventInfo";
static char torsobot_interfaces__srv__RequestControlParams_Event__TYPE_NAME[] = "torsobot_interfaces/srv/RequestControlParams_Event";
static char torsobot_interfaces__srv__RequestControlParams_Request__TYPE_NAME[] = "torsobot_interfaces/srv/RequestControlParams_Request";
static char torsobot_interfaces__srv__RequestControlParams_Response__TYPE_NAME[] = "torsobot_interfaces/srv/RequestControlParams_Response";

// Define type names, field names, and default values
static char torsobot_interfaces__srv__RequestControlParams__FIELD_NAME__request_message[] = "request_message";
static char torsobot_interfaces__srv__RequestControlParams__FIELD_NAME__response_message[] = "response_message";
static char torsobot_interfaces__srv__RequestControlParams__FIELD_NAME__event_message[] = "event_message";

static rosidl_runtime_c__type_description__Field torsobot_interfaces__srv__RequestControlParams__FIELDS[] = {
  {
    {torsobot_interfaces__srv__RequestControlParams__FIELD_NAME__request_message, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {torsobot_interfaces__srv__RequestControlParams_Request__TYPE_NAME, 52, 52},
    },
    {NULL, 0, 0},
  },
  {
    {torsobot_interfaces__srv__RequestControlParams__FIELD_NAME__response_message, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {torsobot_interfaces__srv__RequestControlParams_Response__TYPE_NAME, 53, 53},
    },
    {NULL, 0, 0},
  },
  {
    {torsobot_interfaces__srv__RequestControlParams__FIELD_NAME__event_message, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {torsobot_interfaces__srv__RequestControlParams_Event__TYPE_NAME, 50, 50},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription torsobot_interfaces__srv__RequestControlParams__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
  {
    {torsobot_interfaces__srv__RequestControlParams_Event__TYPE_NAME, 50, 50},
    {NULL, 0, 0},
  },
  {
    {torsobot_interfaces__srv__RequestControlParams_Request__TYPE_NAME, 52, 52},
    {NULL, 0, 0},
  },
  {
    {torsobot_interfaces__srv__RequestControlParams_Response__TYPE_NAME, 53, 53},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
torsobot_interfaces__srv__RequestControlParams__get_type_description(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {torsobot_interfaces__srv__RequestControlParams__TYPE_NAME, 44, 44},
      {torsobot_interfaces__srv__RequestControlParams__FIELDS, 3, 3},
    },
    {torsobot_interfaces__srv__RequestControlParams__REFERENCED_TYPE_DESCRIPTIONS, 5, 5},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = torsobot_interfaces__srv__RequestControlParams_Event__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[3].fields = torsobot_interfaces__srv__RequestControlParams_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[4].fields = torsobot_interfaces__srv__RequestControlParams_Response__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char torsobot_interfaces__srv__RequestControlParams_Request__FIELD_NAME__structure_needs_at_least_one_member[] = "structure_needs_at_least_one_member";

static rosidl_runtime_c__type_description__Field torsobot_interfaces__srv__RequestControlParams_Request__FIELDS[] = {
  {
    {torsobot_interfaces__srv__RequestControlParams_Request__FIELD_NAME__structure_needs_at_least_one_member, 35, 35},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
torsobot_interfaces__srv__RequestControlParams_Request__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {torsobot_interfaces__srv__RequestControlParams_Request__TYPE_NAME, 52, 52},
      {torsobot_interfaces__srv__RequestControlParams_Request__FIELDS, 1, 1},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char torsobot_interfaces__srv__RequestControlParams_Response__FIELD_NAME__desired_torso_pitch[] = "desired_torso_pitch";
static char torsobot_interfaces__srv__RequestControlParams_Response__FIELD_NAME__mot_max_torque[] = "mot_max_torque";
static char torsobot_interfaces__srv__RequestControlParams_Response__FIELD_NAME__kp[] = "kp";
static char torsobot_interfaces__srv__RequestControlParams_Response__FIELD_NAME__ki[] = "ki";
static char torsobot_interfaces__srv__RequestControlParams_Response__FIELD_NAME__kd[] = "kd";

static rosidl_runtime_c__type_description__Field torsobot_interfaces__srv__RequestControlParams_Response__FIELDS[] = {
  {
    {torsobot_interfaces__srv__RequestControlParams_Response__FIELD_NAME__desired_torso_pitch, 19, 19},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {torsobot_interfaces__srv__RequestControlParams_Response__FIELD_NAME__mot_max_torque, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {torsobot_interfaces__srv__RequestControlParams_Response__FIELD_NAME__kp, 2, 2},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {torsobot_interfaces__srv__RequestControlParams_Response__FIELD_NAME__ki, 2, 2},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {torsobot_interfaces__srv__RequestControlParams_Response__FIELD_NAME__kd, 2, 2},
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
torsobot_interfaces__srv__RequestControlParams_Response__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {torsobot_interfaces__srv__RequestControlParams_Response__TYPE_NAME, 53, 53},
      {torsobot_interfaces__srv__RequestControlParams_Response__FIELDS, 5, 5},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char torsobot_interfaces__srv__RequestControlParams_Event__FIELD_NAME__info[] = "info";
static char torsobot_interfaces__srv__RequestControlParams_Event__FIELD_NAME__request[] = "request";
static char torsobot_interfaces__srv__RequestControlParams_Event__FIELD_NAME__response[] = "response";

static rosidl_runtime_c__type_description__Field torsobot_interfaces__srv__RequestControlParams_Event__FIELDS[] = {
  {
    {torsobot_interfaces__srv__RequestControlParams_Event__FIELD_NAME__info, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    },
    {NULL, 0, 0},
  },
  {
    {torsobot_interfaces__srv__RequestControlParams_Event__FIELD_NAME__request, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {torsobot_interfaces__srv__RequestControlParams_Request__TYPE_NAME, 52, 52},
    },
    {NULL, 0, 0},
  },
  {
    {torsobot_interfaces__srv__RequestControlParams_Event__FIELD_NAME__response, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {torsobot_interfaces__srv__RequestControlParams_Response__TYPE_NAME, 53, 53},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription torsobot_interfaces__srv__RequestControlParams_Event__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
  {
    {torsobot_interfaces__srv__RequestControlParams_Request__TYPE_NAME, 52, 52},
    {NULL, 0, 0},
  },
  {
    {torsobot_interfaces__srv__RequestControlParams_Response__TYPE_NAME, 53, 53},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
torsobot_interfaces__srv__RequestControlParams_Event__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {torsobot_interfaces__srv__RequestControlParams_Event__TYPE_NAME, 50, 50},
      {torsobot_interfaces__srv__RequestControlParams_Event__FIELDS, 3, 3},
    },
    {torsobot_interfaces__srv__RequestControlParams_Event__REFERENCED_TYPE_DESCRIPTIONS, 4, 4},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = torsobot_interfaces__srv__RequestControlParams_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[3].fields = torsobot_interfaces__srv__RequestControlParams_Response__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "---\n"
  "float64 desired_torso_pitch\n"
  "float64 mot_max_torque\n"
  "float64 kp\n"
  "float64 ki\n"
  "float64 kd";

static char srv_encoding[] = "srv";
static char implicit_encoding[] = "implicit";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
torsobot_interfaces__srv__RequestControlParams__get_individual_type_description_source(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {torsobot_interfaces__srv__RequestControlParams__TYPE_NAME, 44, 44},
    {srv_encoding, 3, 3},
    {toplevel_type_raw_source, 87, 87},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
torsobot_interfaces__srv__RequestControlParams_Request__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {torsobot_interfaces__srv__RequestControlParams_Request__TYPE_NAME, 52, 52},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
torsobot_interfaces__srv__RequestControlParams_Response__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {torsobot_interfaces__srv__RequestControlParams_Response__TYPE_NAME, 53, 53},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
torsobot_interfaces__srv__RequestControlParams_Event__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {torsobot_interfaces__srv__RequestControlParams_Event__TYPE_NAME, 50, 50},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
torsobot_interfaces__srv__RequestControlParams__get_type_description_sources(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[6];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 6, 6};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *torsobot_interfaces__srv__RequestControlParams__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    sources[3] = *torsobot_interfaces__srv__RequestControlParams_Event__get_individual_type_description_source(NULL);
    sources[4] = *torsobot_interfaces__srv__RequestControlParams_Request__get_individual_type_description_source(NULL);
    sources[5] = *torsobot_interfaces__srv__RequestControlParams_Response__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
torsobot_interfaces__srv__RequestControlParams_Request__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *torsobot_interfaces__srv__RequestControlParams_Request__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
torsobot_interfaces__srv__RequestControlParams_Response__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *torsobot_interfaces__srv__RequestControlParams_Response__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
torsobot_interfaces__srv__RequestControlParams_Event__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[5];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 5, 5};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *torsobot_interfaces__srv__RequestControlParams_Event__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    sources[3] = *torsobot_interfaces__srv__RequestControlParams_Request__get_individual_type_description_source(NULL);
    sources[4] = *torsobot_interfaces__srv__RequestControlParams_Response__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
