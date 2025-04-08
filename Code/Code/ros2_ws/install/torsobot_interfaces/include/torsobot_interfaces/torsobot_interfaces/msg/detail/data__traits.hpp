// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from torsobot_interfaces:msg/Data.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "torsobot_interfaces/msg/data.hpp"


#ifndef TORSOBOT_INTERFACES__MSG__DETAIL__DATA__TRAITS_HPP_
#define TORSOBOT_INTERFACES__MSG__DETAIL__DATA__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "torsobot_interfaces/msg/detail/data__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace torsobot_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const Data & msg,
  std::ostream & out)
{
  out << "{";
  // member: torso_pitch
  {
    out << "torso_pitch: ";
    rosidl_generator_traits::value_to_yaml(msg.torso_pitch, out);
    out << ", ";
  }

  // member: wheel_pitch
  {
    out << "wheel_pitch: ";
    rosidl_generator_traits::value_to_yaml(msg.wheel_pitch, out);
    out << ", ";
  }

  // member: motor_current
  {
    out << "motor_current: ";
    rosidl_generator_traits::value_to_yaml(msg.motor_current, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Data & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: torso_pitch
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "torso_pitch: ";
    rosidl_generator_traits::value_to_yaml(msg.torso_pitch, out);
    out << "\n";
  }

  // member: wheel_pitch
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "wheel_pitch: ";
    rosidl_generator_traits::value_to_yaml(msg.wheel_pitch, out);
    out << "\n";
  }

  // member: motor_current
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "motor_current: ";
    rosidl_generator_traits::value_to_yaml(msg.motor_current, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Data & msg, bool use_flow_style = false)
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
  const torsobot_interfaces::msg::Data & msg,
  std::ostream & out, size_t indentation = 0)
{
  torsobot_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use torsobot_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const torsobot_interfaces::msg::Data & msg)
{
  return torsobot_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<torsobot_interfaces::msg::Data>()
{
  return "torsobot_interfaces::msg::Data";
}

template<>
inline const char * name<torsobot_interfaces::msg::Data>()
{
  return "torsobot_interfaces/msg/Data";
}

template<>
struct has_fixed_size<torsobot_interfaces::msg::Data>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<torsobot_interfaces::msg::Data>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<torsobot_interfaces::msg::Data>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // TORSOBOT_INTERFACES__MSG__DETAIL__DATA__TRAITS_HPP_
