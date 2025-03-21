// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from torsobot_interfaces:msg/Data.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "torsobot_interfaces/msg/data.hpp"


#ifndef TORSOBOT_INTERFACES__MSG__DETAIL__DATA__BUILDER_HPP_
#define TORSOBOT_INTERFACES__MSG__DETAIL__DATA__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "torsobot_interfaces/msg/detail/data__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace torsobot_interfaces
{

namespace msg
{

namespace builder
{

class Init_Data_motor_current
{
public:
  explicit Init_Data_motor_current(::torsobot_interfaces::msg::Data & msg)
  : msg_(msg)
  {}
  ::torsobot_interfaces::msg::Data motor_current(::torsobot_interfaces::msg::Data::_motor_current_type arg)
  {
    msg_.motor_current = std::move(arg);
    return std::move(msg_);
  }

private:
  ::torsobot_interfaces::msg::Data msg_;
};

class Init_Data_wheel_pitch
{
public:
  explicit Init_Data_wheel_pitch(::torsobot_interfaces::msg::Data & msg)
  : msg_(msg)
  {}
  Init_Data_motor_current wheel_pitch(::torsobot_interfaces::msg::Data::_wheel_pitch_type arg)
  {
    msg_.wheel_pitch = std::move(arg);
    return Init_Data_motor_current(msg_);
  }

private:
  ::torsobot_interfaces::msg::Data msg_;
};

class Init_Data_torso_pitch
{
public:
  Init_Data_torso_pitch()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Data_wheel_pitch torso_pitch(::torsobot_interfaces::msg::Data::_torso_pitch_type arg)
  {
    msg_.torso_pitch = std::move(arg);
    return Init_Data_wheel_pitch(msg_);
  }

private:
  ::torsobot_interfaces::msg::Data msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::torsobot_interfaces::msg::Data>()
{
  return torsobot_interfaces::msg::builder::Init_Data_torso_pitch();
}

}  // namespace torsobot_interfaces

#endif  // TORSOBOT_INTERFACES__MSG__DETAIL__DATA__BUILDER_HPP_
