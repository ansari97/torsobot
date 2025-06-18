// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from torsobot_interfaces:msg/TorsobotState.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "torsobot_interfaces/msg/torsobot_state.hpp"


#ifndef TORSOBOT_INTERFACES__MSG__DETAIL__TORSOBOT_STATE__TRAITS_HPP_
#define TORSOBOT_INTERFACES__MSG__DETAIL__TORSOBOT_STATE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "torsobot_interfaces/msg/detail/torsobot_state__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace torsobot_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const TorsobotState & msg,
  std::ostream & out)
{
  out << "{";
  // member: torso_pitch
  {
    out << "torso_pitch: ";
    rosidl_generator_traits::value_to_yaml(msg.torso_pitch, out);
    out << ", ";
  }

  // member: torso_pitch_rate
  {
    out << "torso_pitch_rate: ";
    rosidl_generator_traits::value_to_yaml(msg.torso_pitch_rate, out);
    out << ", ";
  }

  // member: motor_pos
  {
    out << "motor_pos: ";
    rosidl_generator_traits::value_to_yaml(msg.motor_pos, out);
    out << ", ";
  }

  // member: motor_vel
  {
    out << "motor_vel: ";
    rosidl_generator_traits::value_to_yaml(msg.motor_vel, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const TorsobotState & msg,
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

  // member: torso_pitch_rate
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "torso_pitch_rate: ";
    rosidl_generator_traits::value_to_yaml(msg.torso_pitch_rate, out);
    out << "\n";
  }

  // member: motor_pos
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "motor_pos: ";
    rosidl_generator_traits::value_to_yaml(msg.motor_pos, out);
    out << "\n";
  }

  // member: motor_vel
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "motor_vel: ";
    rosidl_generator_traits::value_to_yaml(msg.motor_vel, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const TorsobotState & msg, bool use_flow_style = false)
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
  const torsobot_interfaces::msg::TorsobotState & msg,
  std::ostream & out, size_t indentation = 0)
{
  torsobot_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use torsobot_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const torsobot_interfaces::msg::TorsobotState & msg)
{
  return torsobot_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<torsobot_interfaces::msg::TorsobotState>()
{
  return "torsobot_interfaces::msg::TorsobotState";
}

template<>
inline const char * name<torsobot_interfaces::msg::TorsobotState>()
{
  return "torsobot_interfaces/msg/TorsobotState";
}

template<>
struct has_fixed_size<torsobot_interfaces::msg::TorsobotState>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<torsobot_interfaces::msg::TorsobotState>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<torsobot_interfaces::msg::TorsobotState>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // TORSOBOT_INTERFACES__MSG__DETAIL__TORSOBOT_STATE__TRAITS_HPP_
