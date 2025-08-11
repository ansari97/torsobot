// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from torsobot_interfaces:msg/TorsobotData.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "torsobot_interfaces/msg/torsobot_data.hpp"


#ifndef TORSOBOT_INTERFACES__MSG__DETAIL__TORSOBOT_DATA__TRAITS_HPP_
#define TORSOBOT_INTERFACES__MSG__DETAIL__TORSOBOT_DATA__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "torsobot_interfaces/msg/detail/torsobot_data__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'torsobot_state'
#include "torsobot_interfaces/msg/detail/torsobot_state__traits.hpp"

namespace torsobot_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const TorsobotData & msg,
  std::ostream & out)
{
  out << "{";
  // member: torsobot_state
  {
    out << "torsobot_state: ";
    to_flow_style_yaml(msg.torsobot_state, out);
    out << ", ";
  }

  // member: wheel_torque
  {
    out << "wheel_torque: ";
    rosidl_generator_traits::value_to_yaml(msg.wheel_torque, out);
    out << ", ";
  }

  // member: wheel_cmd_torque
  {
    out << "wheel_cmd_torque: ";
    rosidl_generator_traits::value_to_yaml(msg.wheel_cmd_torque, out);
    out << ", ";
  }

  // member: mot_drv_mode
  {
    out << "mot_drv_mode: ";
    rosidl_generator_traits::value_to_yaml(msg.mot_drv_mode, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const TorsobotData & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: torsobot_state
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "torsobot_state:\n";
    to_block_style_yaml(msg.torsobot_state, out, indentation + 2);
  }

  // member: wheel_torque
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "wheel_torque: ";
    rosidl_generator_traits::value_to_yaml(msg.wheel_torque, out);
    out << "\n";
  }

  // member: wheel_cmd_torque
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "wheel_cmd_torque: ";
    rosidl_generator_traits::value_to_yaml(msg.wheel_cmd_torque, out);
    out << "\n";
  }

  // member: mot_drv_mode
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "mot_drv_mode: ";
    rosidl_generator_traits::value_to_yaml(msg.mot_drv_mode, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const TorsobotData & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace torsobot_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use torsobot_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const torsobot_interfaces::msg::TorsobotData & msg,
  std::ostream & out, size_t indentation = 0)
{
  torsobot_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use torsobot_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const torsobot_interfaces::msg::TorsobotData & msg)
{
  return torsobot_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<torsobot_interfaces::msg::TorsobotData>()
{
  return "torsobot_interfaces::msg::TorsobotData";
}

template<>
inline const char * name<torsobot_interfaces::msg::TorsobotData>()
{
  return "torsobot_interfaces/msg/TorsobotData";
}

template<>
struct has_fixed_size<torsobot_interfaces::msg::TorsobotData>
  : std::integral_constant<bool, has_fixed_size<torsobot_interfaces::msg::TorsobotState>::value> {};

template<>
struct has_bounded_size<torsobot_interfaces::msg::TorsobotData>
  : std::integral_constant<bool, has_bounded_size<torsobot_interfaces::msg::TorsobotState>::value> {};

template<>
struct is_message<torsobot_interfaces::msg::TorsobotData>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // TORSOBOT_INTERFACES__MSG__DETAIL__TORSOBOT_DATA__TRAITS_HPP_
