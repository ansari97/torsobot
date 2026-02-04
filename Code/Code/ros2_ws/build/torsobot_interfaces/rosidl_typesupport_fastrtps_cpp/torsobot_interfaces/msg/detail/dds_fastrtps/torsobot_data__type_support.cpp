// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from torsobot_interfaces:msg/TorsobotData.idl
// generated code does not contain a copyright notice
#include "torsobot_interfaces/msg/detail/torsobot_data__rosidl_typesupport_fastrtps_cpp.hpp"
#include "torsobot_interfaces/msg/detail/torsobot_data__functions.h"
#include "torsobot_interfaces/msg/detail/torsobot_data__struct.hpp"

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
bool cdr_serialize(
  const torsobot_interfaces::msg::TorsobotState &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  torsobot_interfaces::msg::TorsobotState &);
size_t get_serialized_size(
  const torsobot_interfaces::msg::TorsobotState &,
  size_t current_alignment);
size_t
max_serialized_size_TorsobotState(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);
bool cdr_serialize_key(
  const torsobot_interfaces::msg::TorsobotState &,
  eprosima::fastcdr::Cdr &);
size_t get_serialized_size_key(
  const torsobot_interfaces::msg::TorsobotState &,
  size_t current_alignment);
size_t
max_serialized_size_key_TorsobotState(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace torsobot_interfaces


namespace torsobot_interfaces
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{


bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_torsobot_interfaces
cdr_serialize(
  const torsobot_interfaces::msg::TorsobotData & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: torsobot_state
  torsobot_interfaces::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.torsobot_state,
    cdr);

  // Member: wheel_torque
  cdr << ros_message.wheel_torque;

  // Member: wheel_cmd_torque
  cdr << ros_message.wheel_cmd_torque;

  // Member: mot_drv_mode
  cdr << ros_message.mot_drv_mode;

  // Member: torso_pitch_init
  cdr << ros_message.torso_pitch_init;

  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_torsobot_interfaces
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  torsobot_interfaces::msg::TorsobotData & ros_message)
{
  // Member: torsobot_state
  torsobot_interfaces::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.torsobot_state);

  // Member: wheel_torque
  cdr >> ros_message.wheel_torque;

  // Member: wheel_cmd_torque
  cdr >> ros_message.wheel_cmd_torque;

  // Member: mot_drv_mode
  cdr >> ros_message.mot_drv_mode;

  // Member: torso_pitch_init
  cdr >> ros_message.torso_pitch_init;

  return true;
}


size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_torsobot_interfaces
get_serialized_size(
  const torsobot_interfaces::msg::TorsobotData & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: torsobot_state
  current_alignment +=
    torsobot_interfaces::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.torsobot_state, current_alignment);

  // Member: wheel_torque
  {
    size_t item_size = sizeof(ros_message.wheel_torque);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: wheel_cmd_torque
  {
    size_t item_size = sizeof(ros_message.wheel_cmd_torque);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: mot_drv_mode
  {
    size_t item_size = sizeof(ros_message.mot_drv_mode);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: torso_pitch_init
  {
    size_t item_size = sizeof(ros_message.torso_pitch_init);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}


size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_torsobot_interfaces
max_serialized_size_TorsobotData(
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

  // Member: torsobot_state
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        torsobot_interfaces::msg::typesupport_fastrtps_cpp::max_serialized_size_TorsobotState(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }
  // Member: wheel_torque
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // Member: wheel_cmd_torque
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // Member: mot_drv_mode
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // Member: torso_pitch_init
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
    using DataType = torsobot_interfaces::msg::TorsobotData;
    is_plain =
      (
      offsetof(DataType, torso_pitch_init) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_torsobot_interfaces
cdr_serialize_key(
  const torsobot_interfaces::msg::TorsobotData & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: torsobot_state
  torsobot_interfaces::msg::typesupport_fastrtps_cpp::cdr_serialize_key(
    ros_message.torsobot_state,
    cdr);

  // Member: wheel_torque
  cdr << ros_message.wheel_torque;

  // Member: wheel_cmd_torque
  cdr << ros_message.wheel_cmd_torque;

  // Member: mot_drv_mode
  cdr << ros_message.mot_drv_mode;

  // Member: torso_pitch_init
  cdr << ros_message.torso_pitch_init;

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_torsobot_interfaces
get_serialized_size_key(
  const torsobot_interfaces::msg::TorsobotData & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: torsobot_state
  current_alignment +=
    torsobot_interfaces::msg::typesupport_fastrtps_cpp::get_serialized_size_key(
    ros_message.torsobot_state, current_alignment);

  // Member: wheel_torque
  {
    size_t item_size = sizeof(ros_message.wheel_torque);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: wheel_cmd_torque
  {
    size_t item_size = sizeof(ros_message.wheel_cmd_torque);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: mot_drv_mode
  {
    size_t item_size = sizeof(ros_message.mot_drv_mode);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: torso_pitch_init
  {
    size_t item_size = sizeof(ros_message.torso_pitch_init);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_torsobot_interfaces
max_serialized_size_key_TorsobotData(
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

  // Member: torsobot_state
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        torsobot_interfaces::msg::typesupport_fastrtps_cpp::max_serialized_size_key_TorsobotState(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Member: wheel_torque
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: wheel_cmd_torque
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: mot_drv_mode
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: torso_pitch_init
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
    using DataType = torsobot_interfaces::msg::TorsobotData;
    is_plain =
      (
      offsetof(DataType, torso_pitch_init) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}


static bool _TorsobotData__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const torsobot_interfaces::msg::TorsobotData *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _TorsobotData__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<torsobot_interfaces::msg::TorsobotData *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _TorsobotData__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const torsobot_interfaces::msg::TorsobotData *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _TorsobotData__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_TorsobotData(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _TorsobotData__callbacks = {
  "torsobot_interfaces::msg",
  "TorsobotData",
  _TorsobotData__cdr_serialize,
  _TorsobotData__cdr_deserialize,
  _TorsobotData__get_serialized_size,
  _TorsobotData__max_serialized_size,
  nullptr
};

static rosidl_message_type_support_t _TorsobotData__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_TorsobotData__callbacks,
  get_message_typesupport_handle_function,
  &torsobot_interfaces__msg__TorsobotData__get_type_hash,
  &torsobot_interfaces__msg__TorsobotData__get_type_description,
  &torsobot_interfaces__msg__TorsobotData__get_type_description_sources,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace torsobot_interfaces

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_torsobot_interfaces
const rosidl_message_type_support_t *
get_message_type_support_handle<torsobot_interfaces::msg::TorsobotData>()
{
  return &torsobot_interfaces::msg::typesupport_fastrtps_cpp::_TorsobotData__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, torsobot_interfaces, msg, TorsobotData)() {
  return &torsobot_interfaces::msg::typesupport_fastrtps_cpp::_TorsobotData__handle;
}

#ifdef __cplusplus
}
#endif
