// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from torsobot_interfaces:srv/RequestControlParams.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "torsobot_interfaces/srv/request_control_params.hpp"


#ifndef TORSOBOT_INTERFACES__SRV__DETAIL__REQUEST_CONTROL_PARAMS__BUILDER_HPP_
#define TORSOBOT_INTERFACES__SRV__DETAIL__REQUEST_CONTROL_PARAMS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "torsobot_interfaces/srv/detail/request_control_params__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace torsobot_interfaces
{

namespace srv
{


}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::torsobot_interfaces::srv::RequestControlParams_Request>()
{
  return ::torsobot_interfaces::srv::RequestControlParams_Request(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace torsobot_interfaces


namespace torsobot_interfaces
{

namespace srv
{

namespace builder
{

class Init_RequestControlParams_Response_kd
{
public:
  explicit Init_RequestControlParams_Response_kd(::torsobot_interfaces::srv::RequestControlParams_Response & msg)
  : msg_(msg)
  {}
  ::torsobot_interfaces::srv::RequestControlParams_Response kd(::torsobot_interfaces::srv::RequestControlParams_Response::_kd_type arg)
  {
    msg_.kd = std::move(arg);
    return std::move(msg_);
  }

private:
  ::torsobot_interfaces::srv::RequestControlParams_Response msg_;
};

class Init_RequestControlParams_Response_ki
{
public:
  explicit Init_RequestControlParams_Response_ki(::torsobot_interfaces::srv::RequestControlParams_Response & msg)
  : msg_(msg)
  {}
  Init_RequestControlParams_Response_kd ki(::torsobot_interfaces::srv::RequestControlParams_Response::_ki_type arg)
  {
    msg_.ki = std::move(arg);
    return Init_RequestControlParams_Response_kd(msg_);
  }

private:
  ::torsobot_interfaces::srv::RequestControlParams_Response msg_;
};

class Init_RequestControlParams_Response_kp
{
public:
  explicit Init_RequestControlParams_Response_kp(::torsobot_interfaces::srv::RequestControlParams_Response & msg)
  : msg_(msg)
  {}
  Init_RequestControlParams_Response_ki kp(::torsobot_interfaces::srv::RequestControlParams_Response::_kp_type arg)
  {
    msg_.kp = std::move(arg);
    return Init_RequestControlParams_Response_ki(msg_);
  }

private:
  ::torsobot_interfaces::srv::RequestControlParams_Response msg_;
};

class Init_RequestControlParams_Response_mot_max_torque
{
public:
  explicit Init_RequestControlParams_Response_mot_max_torque(::torsobot_interfaces::srv::RequestControlParams_Response & msg)
  : msg_(msg)
  {}
  Init_RequestControlParams_Response_kp mot_max_torque(::torsobot_interfaces::srv::RequestControlParams_Response::_mot_max_torque_type arg)
  {
    msg_.mot_max_torque = std::move(arg);
    return Init_RequestControlParams_Response_kp(msg_);
  }

private:
  ::torsobot_interfaces::srv::RequestControlParams_Response msg_;
};

class Init_RequestControlParams_Response_desired_torso_pitch
{
public:
  Init_RequestControlParams_Response_desired_torso_pitch()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RequestControlParams_Response_mot_max_torque desired_torso_pitch(::torsobot_interfaces::srv::RequestControlParams_Response::_desired_torso_pitch_type arg)
  {
    msg_.desired_torso_pitch = std::move(arg);
    return Init_RequestControlParams_Response_mot_max_torque(msg_);
  }

private:
  ::torsobot_interfaces::srv::RequestControlParams_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::torsobot_interfaces::srv::RequestControlParams_Response>()
{
  return torsobot_interfaces::srv::builder::Init_RequestControlParams_Response_desired_torso_pitch();
}

}  // namespace torsobot_interfaces


namespace torsobot_interfaces
{

namespace srv
{

namespace builder
{

class Init_RequestControlParams_Event_response
{
public:
  explicit Init_RequestControlParams_Event_response(::torsobot_interfaces::srv::RequestControlParams_Event & msg)
  : msg_(msg)
  {}
  ::torsobot_interfaces::srv::RequestControlParams_Event response(::torsobot_interfaces::srv::RequestControlParams_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::torsobot_interfaces::srv::RequestControlParams_Event msg_;
};

class Init_RequestControlParams_Event_request
{
public:
  explicit Init_RequestControlParams_Event_request(::torsobot_interfaces::srv::RequestControlParams_Event & msg)
  : msg_(msg)
  {}
  Init_RequestControlParams_Event_response request(::torsobot_interfaces::srv::RequestControlParams_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_RequestControlParams_Event_response(msg_);
  }

private:
  ::torsobot_interfaces::srv::RequestControlParams_Event msg_;
};

class Init_RequestControlParams_Event_info
{
public:
  Init_RequestControlParams_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RequestControlParams_Event_request info(::torsobot_interfaces::srv::RequestControlParams_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_RequestControlParams_Event_request(msg_);
  }

private:
  ::torsobot_interfaces::srv::RequestControlParams_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::torsobot_interfaces::srv::RequestControlParams_Event>()
{
  return torsobot_interfaces::srv::builder::Init_RequestControlParams_Event_info();
}

}  // namespace torsobot_interfaces

#endif  // TORSOBOT_INTERFACES__SRV__DETAIL__REQUEST_CONTROL_PARAMS__BUILDER_HPP_
