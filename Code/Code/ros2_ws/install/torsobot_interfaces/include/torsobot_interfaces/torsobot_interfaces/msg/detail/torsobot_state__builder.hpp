// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from torsobot_interfaces:msg/TorsobotState.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "torsobot_interfaces/msg/torsobot_state.hpp"


#ifndef TORSOBOT_INTERFACES__MSG__DETAIL__TORSOBOT_STATE__BUILDER_HPP_
#define TORSOBOT_INTERFACES__MSG__DETAIL__TORSOBOT_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "torsobot_interfaces/msg/detail/torsobot_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace torsobot_interfaces
{

namespace msg
{

namespace builder
{

class Init_TorsobotState_wheel_vel
{
public:
  explicit Init_TorsobotState_wheel_vel(::torsobot_interfaces::msg::TorsobotState & msg)
  : msg_(msg)
  {}
  ::torsobot_interfaces::msg::TorsobotState wheel_vel(::torsobot_interfaces::msg::TorsobotState::_wheel_vel_type arg)
  {
    msg_.wheel_vel = std::move(arg);
    return std::move(msg_);
  }

private:
  ::torsobot_interfaces::msg::TorsobotState msg_;
};

class Init_TorsobotState_wheel_pos
{
public:
  explicit Init_TorsobotState_wheel_pos(::torsobot_interfaces::msg::TorsobotState & msg)
  : msg_(msg)
  {}
  Init_TorsobotState_wheel_vel wheel_pos(::torsobot_interfaces::msg::TorsobotState::_wheel_pos_type arg)
  {
    msg_.wheel_pos = std::move(arg);
    return Init_TorsobotState_wheel_vel(msg_);
  }

private:
  ::torsobot_interfaces::msg::TorsobotState msg_;
};

class Init_TorsobotState_torso_pitch_rate
{
public:
  explicit Init_TorsobotState_torso_pitch_rate(::torsobot_interfaces::msg::TorsobotState & msg)
  : msg_(msg)
  {}
  Init_TorsobotState_wheel_pos torso_pitch_rate(::torsobot_interfaces::msg::TorsobotState::_torso_pitch_rate_type arg)
  {
    msg_.torso_pitch_rate = std::move(arg);
    return Init_TorsobotState_wheel_pos(msg_);
  }

private:
  ::torsobot_interfaces::msg::TorsobotState msg_;
};

class Init_TorsobotState_torso_pitch
{
public:
  Init_TorsobotState_torso_pitch()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TorsobotState_torso_pitch_rate torso_pitch(::torsobot_interfaces::msg::TorsobotState::_torso_pitch_type arg)
  {
    msg_.torso_pitch = std::move(arg);
    return Init_TorsobotState_torso_pitch_rate(msg_);
  }

private:
  ::torsobot_interfaces::msg::TorsobotState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::torsobot_interfaces::msg::TorsobotState>()
{
  return torsobot_interfaces::msg::builder::Init_TorsobotState_torso_pitch();
}

}  // namespace torsobot_interfaces

#endif  // TORSOBOT_INTERFACES__MSG__DETAIL__TORSOBOT_STATE__BUILDER_HPP_
