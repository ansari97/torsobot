// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from torsobot_interfaces:msg/TorsobotData.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "torsobot_interfaces/msg/torsobot_data.hpp"


#ifndef TORSOBOT_INTERFACES__MSG__DETAIL__TORSOBOT_DATA__BUILDER_HPP_
#define TORSOBOT_INTERFACES__MSG__DETAIL__TORSOBOT_DATA__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "torsobot_interfaces/msg/detail/torsobot_data__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace torsobot_interfaces
{

namespace msg
{

namespace builder
{

class Init_TorsobotData_motor_drv_mode
{
public:
  explicit Init_TorsobotData_motor_drv_mode(::torsobot_interfaces::msg::TorsobotData & msg)
  : msg_(msg)
  {}
  ::torsobot_interfaces::msg::TorsobotData motor_drv_mode(::torsobot_interfaces::msg::TorsobotData::_motor_drv_mode_type arg)
  {
    msg_.motor_drv_mode = std::move(arg);
    return std::move(msg_);
  }

private:
  ::torsobot_interfaces::msg::TorsobotData msg_;
};

class Init_TorsobotData_motor_torque
{
public:
  explicit Init_TorsobotData_motor_torque(::torsobot_interfaces::msg::TorsobotData & msg)
  : msg_(msg)
  {}
  Init_TorsobotData_motor_drv_mode motor_torque(::torsobot_interfaces::msg::TorsobotData::_motor_torque_type arg)
  {
    msg_.motor_torque = std::move(arg);
    return Init_TorsobotData_motor_drv_mode(msg_);
  }

private:
  ::torsobot_interfaces::msg::TorsobotData msg_;
};

class Init_TorsobotData_motor_vel
{
public:
  explicit Init_TorsobotData_motor_vel(::torsobot_interfaces::msg::TorsobotData & msg)
  : msg_(msg)
  {}
  Init_TorsobotData_motor_torque motor_vel(::torsobot_interfaces::msg::TorsobotData::_motor_vel_type arg)
  {
    msg_.motor_vel = std::move(arg);
    return Init_TorsobotData_motor_torque(msg_);
  }

private:
  ::torsobot_interfaces::msg::TorsobotData msg_;
};

class Init_TorsobotData_motor_pos
{
public:
  explicit Init_TorsobotData_motor_pos(::torsobot_interfaces::msg::TorsobotData & msg)
  : msg_(msg)
  {}
  Init_TorsobotData_motor_vel motor_pos(::torsobot_interfaces::msg::TorsobotData::_motor_pos_type arg)
  {
    msg_.motor_pos = std::move(arg);
    return Init_TorsobotData_motor_vel(msg_);
  }

private:
  ::torsobot_interfaces::msg::TorsobotData msg_;
};

class Init_TorsobotData_torso_pitch_rate
{
public:
  explicit Init_TorsobotData_torso_pitch_rate(::torsobot_interfaces::msg::TorsobotData & msg)
  : msg_(msg)
  {}
  Init_TorsobotData_motor_pos torso_pitch_rate(::torsobot_interfaces::msg::TorsobotData::_torso_pitch_rate_type arg)
  {
    msg_.torso_pitch_rate = std::move(arg);
    return Init_TorsobotData_motor_pos(msg_);
  }

private:
  ::torsobot_interfaces::msg::TorsobotData msg_;
};

class Init_TorsobotData_torso_pitch
{
public:
  Init_TorsobotData_torso_pitch()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TorsobotData_torso_pitch_rate torso_pitch(::torsobot_interfaces::msg::TorsobotData::_torso_pitch_type arg)
  {
    msg_.torso_pitch = std::move(arg);
    return Init_TorsobotData_torso_pitch_rate(msg_);
  }

private:
  ::torsobot_interfaces::msg::TorsobotData msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::torsobot_interfaces::msg::TorsobotData>()
{
  return torsobot_interfaces::msg::builder::Init_TorsobotData_torso_pitch();
}

}  // namespace torsobot_interfaces

#endif  // TORSOBOT_INTERFACES__MSG__DETAIL__TORSOBOT_DATA__BUILDER_HPP_
