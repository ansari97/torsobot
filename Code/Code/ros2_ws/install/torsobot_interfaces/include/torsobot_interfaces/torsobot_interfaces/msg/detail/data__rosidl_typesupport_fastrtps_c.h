// generated from rosidl_typesupport_fastrtps_c/resource/idl__rosidl_typesupport_fastrtps_c.h.em
// with input from torsobot_interfaces:msg/Data.idl
// generated code does not contain a copyright notice
#ifndef TORSOBOT_INTERFACES__MSG__DETAIL__DATA__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
#define TORSOBOT_INTERFACES__MSG__DETAIL__DATA__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_


#include <stddef.h>
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "torsobot_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "torsobot_interfaces/msg/detail/data__struct.h"
#include "fastcdr/Cdr.h"

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_torsobot_interfaces
bool cdr_serialize_torsobot_interfaces__msg__Data(
  const torsobot_interfaces__msg__Data * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_torsobot_interfaces
bool cdr_deserialize_torsobot_interfaces__msg__Data(
  eprosima::fastcdr::Cdr &,
  torsobot_interfaces__msg__Data * ros_message);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_torsobot_interfaces
size_t get_serialized_size_torsobot_interfaces__msg__Data(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_torsobot_interfaces
size_t max_serialized_size_torsobot_interfaces__msg__Data(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_torsobot_interfaces
bool cdr_serialize_key_torsobot_interfaces__msg__Data(
  const torsobot_interfaces__msg__Data * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_torsobot_interfaces
size_t get_serialized_size_key_torsobot_interfaces__msg__Data(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_torsobot_interfaces
size_t max_serialized_size_key_torsobot_interfaces__msg__Data(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_torsobot_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, torsobot_interfaces, msg, Data)();

#ifdef __cplusplus
}
#endif

#endif  // TORSOBOT_INTERFACES__MSG__DETAIL__DATA__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
