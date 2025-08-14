// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from torsobot_interfaces:msg/TorsobotState.idl
// generated code does not contain a copyright notice
#include "torsobot_interfaces/msg/detail/torsobot_state__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <cstddef>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/serialization_helpers.hpp"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "torsobot_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "torsobot_interfaces/msg/detail/torsobot_state__struct.h"
#include "torsobot_interfaces/msg/detail/torsobot_state__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif


// forward declare type support functions


using _TorsobotState__ros_msg_type = torsobot_interfaces__msg__TorsobotState;


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_torsobot_interfaces
bool cdr_serialize_torsobot_interfaces__msg__TorsobotState(
  const torsobot_interfaces__msg__TorsobotState * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: torso_pitch
  {
    cdr << ros_message->torso_pitch;
  }

  // Field name: torso_pitch_rate
  {
    cdr << ros_message->torso_pitch_rate;
  }

  // Field name: wheel_pos
  {
    cdr << ros_message->wheel_pos;
  }

  // Field name: wheel_vel
  {
    cdr << ros_message->wheel_vel;
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_torsobot_interfaces
bool cdr_deserialize_torsobot_interfaces__msg__TorsobotState(
  eprosima::fastcdr::Cdr & cdr,
  torsobot_interfaces__msg__TorsobotState * ros_message)
{
  // Field name: torso_pitch
  {
    cdr >> ros_message->torso_pitch;
  }

  // Field name: torso_pitch_rate
  {
    cdr >> ros_message->torso_pitch_rate;
  }

  // Field name: wheel_pos
  {
    cdr >> ros_message->wheel_pos;
  }

  // Field name: wheel_vel
  {
    cdr >> ros_message->wheel_vel;
  }

  return true;
}  // NOLINT(readability/fn_size)


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_torsobot_interfaces
size_t get_serialized_size_torsobot_interfaces__msg__TorsobotState(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _TorsobotState__ros_msg_type * ros_message = static_cast<const _TorsobotState__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: torso_pitch
  {
    size_t item_size = sizeof(ros_message->torso_pitch);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: torso_pitch_rate
  {
    size_t item_size = sizeof(ros_message->torso_pitch_rate);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: wheel_pos
  {
    size_t item_size = sizeof(ros_message->wheel_pos);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: wheel_vel
  {
    size_t item_size = sizeof(ros_message->wheel_vel);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_torsobot_interfaces
size_t max_serialized_size_torsobot_interfaces__msg__TorsobotState(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // Field name: torso_pitch
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Field name: torso_pitch_rate
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Field name: wheel_pos
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Field name: wheel_vel
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }


  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = torsobot_interfaces__msg__TorsobotState;
    is_plain =
      (
      offsetof(DataType, wheel_vel) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_torsobot_interfaces
bool cdr_serialize_key_torsobot_interfaces__msg__TorsobotState(
  const torsobot_interfaces__msg__TorsobotState * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: torso_pitch
  {
    cdr << ros_message->torso_pitch;
  }

  // Field name: torso_pitch_rate
  {
    cdr << ros_message->torso_pitch_rate;
  }

  // Field name: wheel_pos
  {
    cdr << ros_message->wheel_pos;
  }

  // Field name: wheel_vel
  {
    cdr << ros_message->wheel_vel;
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_torsobot_interfaces
size_t get_serialized_size_key_torsobot_interfaces__msg__TorsobotState(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _TorsobotState__ros_msg_type * ros_message = static_cast<const _TorsobotState__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;

  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: torso_pitch
  {
    size_t item_size = sizeof(ros_message->torso_pitch);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: torso_pitch_rate
  {
    size_t item_size = sizeof(ros_message->torso_pitch_rate);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: wheel_pos
  {
    size_t item_size = sizeof(ros_message->wheel_pos);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: wheel_vel
  {
    size_t item_size = sizeof(ros_message->wheel_vel);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_torsobot_interfaces
size_t max_serialized_size_key_torsobot_interfaces__msg__TorsobotState(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;
  // Field name: torso_pitch
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Field name: torso_pitch_rate
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Field name: wheel_pos
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Field name: wheel_vel
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = torsobot_interfaces__msg__TorsobotState;
    is_plain =
      (
      offsetof(DataType, wheel_vel) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}


static bool _TorsobotState__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const torsobot_interfaces__msg__TorsobotState * ros_message = static_cast<const torsobot_interfaces__msg__TorsobotState *>(untyped_ros_message);
  (void)ros_message;
  return cdr_serialize_torsobot_interfaces__msg__TorsobotState(ros_message, cdr);
}

static bool _TorsobotState__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  torsobot_interfaces__msg__TorsobotState * ros_message = static_cast<torsobot_interfaces__msg__TorsobotState *>(untyped_ros_message);
  (void)ros_message;
  return cdr_deserialize_torsobot_interfaces__msg__TorsobotState(cdr, ros_message);
}

static uint32_t _TorsobotState__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_torsobot_interfaces__msg__TorsobotState(
      untyped_ros_message, 0));
}

static size_t _TorsobotState__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_torsobot_interfaces__msg__TorsobotState(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_TorsobotState = {
  "torsobot_interfaces::msg",
  "TorsobotState",
  _TorsobotState__cdr_serialize,
  _TorsobotState__cdr_deserialize,
  _TorsobotState__get_serialized_size,
  _TorsobotState__max_serialized_size,
  nullptr
};

static rosidl_message_type_support_t _TorsobotState__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_TorsobotState,
  get_message_typesupport_handle_function,
  &torsobot_interfaces__msg__TorsobotState__get_type_hash,
  &torsobot_interfaces__msg__TorsobotState__get_type_description,
  &torsobot_interfaces__msg__TorsobotState__get_type_description_sources,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, torsobot_interfaces, msg, TorsobotState)() {
  return &_TorsobotState__type_support;
}

#if defined(__cplusplus)
}
#endif
