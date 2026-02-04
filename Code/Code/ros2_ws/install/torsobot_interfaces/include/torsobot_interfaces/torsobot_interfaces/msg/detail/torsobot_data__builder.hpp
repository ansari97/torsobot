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

class Init_TorsobotData_torso_pitch_init
{
public:
  explicit Init_TorsobotData_torso_pitch_init(::torsobot_interfaces::msg::TorsobotData & msg)
  : msg_(msg)
  {}
  ::torsobot_interfaces::msg::TorsobotData torso_pitch_init(::torsobot_interfaces::msg::TorsobotData::_torso_pitch_init_type arg)
  {
    msg_.torso_pitch_init = std::move(arg);
    return std::move(msg_);
  }

private:
  ::torsobot_interfaces::msg::TorsobotData msg_;
};

class Init_TorsobotData_mot_drv_mode
{
public:
  explicit Init_TorsobotData_mot_drv_mode(::torsobot_interfaces::msg::TorsobotData & msg)
  : msg_(msg)
  {}
  Init_TorsobotData_torso_pitch_init mot_drv_mode(::torsobot_interfaces::msg::TorsobotData::_mot_drv_mode_type arg)
  {
    msg_.mot_drv_mode = std::move(arg);
    return Init_TorsobotData_torso_pitch_init(msg_);
  }

private:
  ::torsobot_interfaces::msg::TorsobotData msg_;
};

class Init_TorsobotData_wheel_cmd_torque
{
public:
  explicit Init_TorsobotData_wheel_cmd_torque(::torsobot_interfaces::msg::TorsobotData & msg)
  : msg_(msg)
  {}
  Init_TorsobotData_mot_drv_mode wheel_cmd_torque(::torsobot_interfaces::msg::TorsobotData::_wheel_cmd_torque_type arg)
  {
    msg_.wheel_cmd_torque = std::move(arg);
    return Init_TorsobotData_mot_drv_mode(msg_);
  }

private:
  ::torsobot_interfaces::msg::TorsobotData msg_;
};

class Init_TorsobotData_wheel_torque
{
public:
  explicit Init_TorsobotData_wheel_torque(::torsobot_interfaces::msg::TorsobotData & msg)
  : msg_(msg)
  {}
  Init_TorsobotData_wheel_cmd_torque wheel_torque(::torsobot_interfaces::msg::TorsobotData::_wheel_torque_type arg)
  {
    msg_.wheel_torque = std::move(arg);
    return Init_TorsobotData_wheel_cmd_torque(msg_);
  }

private:
  ::torsobot_interfaces::msg::TorsobotData msg_;
};

class Init_TorsobotData_torsobot_state
{
public:
  Init_TorsobotData_torsobot_state()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TorsobotData_wheel_torque torsobot_state(::torsobot_interfaces::msg::TorsobotData::_torsobot_state_type arg)
  {
    msg_.torsobot_state = std::move(arg);
    return Init_TorsobotData_wheel_torque(msg_);
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
  return torsobot_interfaces::msg::builder::Init_TorsobotData_torsobot_state();
}

}  // namespace torsobot_interfaces

#endif  // TORSOBOT_INTERFACES__MSG__DETAIL__TORSOBOT_DATA__BUILDER_HPP_
