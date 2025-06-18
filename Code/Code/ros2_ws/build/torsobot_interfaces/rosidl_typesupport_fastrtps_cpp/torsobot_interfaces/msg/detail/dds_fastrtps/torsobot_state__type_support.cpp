// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from torsobot_interfaces:msg/TorsobotState.idl
// generated code does not contain a copyright notice
#include "torsobot_interfaces/msg/detail/torsobot_state__rosidl_typesupport_fastrtps_cpp.hpp"
#include "torsobot_interfaces/msg/detail/torsobot_state__functions.h"
#include "torsobot_interfaces/msg/detail/torsobot_state__struct.hpp"

#include <cstddef>
#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/serialization_helpers.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions

namespace torsobot_interfaces
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{


bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_torsobot_interfaces
cdr_serialize(
  const torsobot_interfaces::msg::TorsobotState & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: torso_pitch
  cdr << ros_message.torso_pitch;

  // Member: torso_pitch_rate
  cdr << ros_message.torso_pitch_rate;

  // Member: motor_pos
  cdr << ros_message.motor_pos;

  // Member: motor_vel
  cdr << ros_message.motor_vel;

  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_torsobot_interfaces
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  torsobot_interfaces::msg::TorsobotState & ros_message)
{
  // Member: torso_pitch
  cdr >> ros_message.torso_pitch;

  // Member: torso_pitch_rate
  cdr >> ros_message.torso_pitch_rate;

  // Member: motor_pos
  cdr >> ros_message.motor_pos;

  // Member: motor_vel
  cdr >> ros_message.motor_vel;

  return true;
}


size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_torsobot_interfaces
get_serialized_size(
  const torsobot_interfaces::msg::TorsobotState & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: torso_pitch
  {
    size_t item_size = sizeof(ros_message.torso_pitch);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: torso_pitch_rate
  {
    size_t item_size = sizeof(ros_message.torso_pitch_rate);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: motor_pos
  {
    size_t item_size = sizeof(ros_message.motor_pos);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: motor_vel
  {
    size_t item_size = sizeof(ros_message.motor_vel);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}


size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_torsobot_interfaces
max_serialized_size_TorsobotState(
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

  // Member: torso_pitch
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // Member: torso_pitch_rate
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // Member: motor_pos
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // Member: motor_vel
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
    using DataType = torsobot_interfaces::msg::TorsobotState;
    is_plain =
      (
      offsetof(DataType, motor_vel) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_torsobot_interfaces
cdr_serialize_key(
  const torsobot_interfaces::msg::TorsobotState & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: torso_pitch
  cdr << ros_message.torso_pitch;

  // Member: torso_pitch_rate
  cdr << ros_message.torso_pitch_rate;

  // Member: motor_pos
  cdr << ros_message.motor_pos;

  // Member: motor_vel
  cdr << ros_message.motor_vel;

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_torsobot_interfaces
get_serialized_size_key(
  const torsobot_interfaces::msg::TorsobotState & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: torso_pitch
  {
    size_t item_size = sizeof(ros_message.torso_pitch);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: torso_pitch_rate
  {
    size_t item_size = sizeof(ros_message.torso_pitch_rate);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: motor_pos
  {
    size_t item_size = sizeof(ros_message.motor_pos);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: motor_vel
  {
    size_t item_size = sizeof(ros_message.motor_vel);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_torsobot_interfaces
max_serialized_size_key_TorsobotState(
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

  // Member: torso_pitch
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: torso_pitch_rate
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: motor_pos
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: motor_vel
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
    using DataType = torsobot_interfaces::msg::TorsobotState;
    is_plain =
      (
      offsetof(DataType, motor_vel) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}


static bool _TorsobotState__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const torsobot_interfaces::msg::TorsobotState *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _TorsobotState__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<torsobot_interfaces::msg::TorsobotState *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _TorsobotState__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const torsobot_interfaces::msg::TorsobotState *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _TorsobotState__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_TorsobotState(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _TorsobotState__callbacks = {
  "torsobot_interfaces::msg",
  "TorsobotState",
  _TorsobotState__cdr_serialize,
  _TorsobotState__cdr_deserialize,
  _TorsobotState__get_serialized_size,
  _TorsobotState__max_serialized_size,
  nullptr
};

static rosidl_message_type_support_t _TorsobotState__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_TorsobotState__callbacks,
  get_message_typesupport_handle_function,
  &torsobot_interfaces__msg__TorsobotState__get_type_hash,
  &torsobot_interfaces__msg__TorsobotState__get_type_description,
  &torsobot_interfaces__msg__TorsobotState__get_type_description_sources,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace torsobot_interfaces

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_torsobot_interfaces
const rosidl_message_type_support_t *
get_message_type_support_handle<torsobot_interfaces::msg::TorsobotState>()
{
  return &torsobot_interfaces::msg::typesupport_fastrtps_cpp::_TorsobotState__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, torsobot_interfaces, msg, TorsobotState)() {
  return &torsobot_interfaces::msg::typesupport_fastrtps_cpp::_TorsobotState__handle;
}

#ifdef __cplusplus
}
#endif
