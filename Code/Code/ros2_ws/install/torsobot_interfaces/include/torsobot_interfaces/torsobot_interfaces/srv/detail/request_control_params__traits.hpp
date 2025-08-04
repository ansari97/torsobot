// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from torsobot_interfaces:srv/RequestControlParams.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "torsobot_interfaces/srv/request_control_params.hpp"


#ifndef TORSOBOT_INTERFACES__SRV__DETAIL__REQUEST_CONTROL_PARAMS__TRAITS_HPP_
#define TORSOBOT_INTERFACES__SRV__DETAIL__REQUEST_CONTROL_PARAMS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "torsobot_interfaces/srv/detail/request_control_params__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace torsobot_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const RequestControlParams_Request & msg,
  std::ostream & out)
{
  (void)msg;
  out << "null";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const RequestControlParams_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  (void)msg;
  (void)indentation;
  out << "null\n";
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const RequestControlParams_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace torsobot_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use torsobot_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const torsobot_interfaces::srv::RequestControlParams_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  torsobot_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use torsobot_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const torsobot_interfaces::srv::RequestControlParams_Request & msg)
{
  return torsobot_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<torsobot_interfaces::srv::RequestControlParams_Request>()
{
  return "torsobot_interfaces::srv::RequestControlParams_Request";
}

template<>
inline const char * name<torsobot_interfaces::srv::RequestControlParams_Request>()
{
  return "torsobot_interfaces/srv/RequestControlParams_Request";
}

template<>
struct has_fixed_size<torsobot_interfaces::srv::RequestControlParams_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<torsobot_interfaces::srv::RequestControlParams_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<torsobot_interfaces::srv::RequestControlParams_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace torsobot_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const RequestControlParams_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: desired_torso_pitch
  {
    out << "desired_torso_pitch: ";
    rosidl_generator_traits::value_to_yaml(msg.desired_torso_pitch, out);
    out << ", ";
  }

  // member: mot_max_torque
  {
    out << "mot_max_torque: ";
    rosidl_generator_traits::value_to_yaml(msg.mot_max_torque, out);
    out << ", ";
  }

  // member: kp
  {
    out << "kp: ";
    rosidl_generator_traits::value_to_yaml(msg.kp, out);
    out << ", ";
  }

  // member: ki
  {
    out << "ki: ";
    rosidl_generator_traits::value_to_yaml(msg.ki, out);
    out << ", ";
  }

  // member: kd
  {
    out << "kd: ";
    rosidl_generator_traits::value_to_yaml(msg.kd, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const RequestControlParams_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: desired_torso_pitch
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "desired_torso_pitch: ";
    rosidl_generator_traits::value_to_yaml(msg.desired_torso_pitch, out);
    out << "\n";
  }

  // member: mot_max_torque
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "mot_max_torque: ";
    rosidl_generator_traits::value_to_yaml(msg.mot_max_torque, out);
    out << "\n";
  }

  // member: kp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "kp: ";
    rosidl_generator_traits::value_to_yaml(msg.kp, out);
    out << "\n";
  }

  // member: ki
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ki: ";
    rosidl_generator_traits::value_to_yaml(msg.ki, out);
    out << "\n";
  }

  // member: kd
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "kd: ";
    rosidl_generator_traits::value_to_yaml(msg.kd, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const RequestControlParams_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace torsobot_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use torsobot_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const torsobot_interfaces::srv::RequestControlParams_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  torsobot_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use torsobot_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const torsobot_interfaces::srv::RequestControlParams_Response & msg)
{
  return torsobot_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<torsobot_interfaces::srv::RequestControlParams_Response>()
{
  return "torsobot_interfaces::srv::RequestControlParams_Response";
}

template<>
inline const char * name<torsobot_interfaces::srv::RequestControlParams_Response>()
{
  return "torsobot_interfaces/srv/RequestControlParams_Response";
}

template<>
struct has_fixed_size<torsobot_interfaces::srv::RequestControlParams_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<torsobot_interfaces::srv::RequestControlParams_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<torsobot_interfaces::srv::RequestControlParams_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__traits.hpp"

namespace torsobot_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const RequestControlParams_Event & msg,
  std::ostream & out)
{
  out << "{";
  // member: info
  {
    out << "info: ";
    to_flow_style_yaml(msg.info, out);
    out << ", ";
  }

  // member: request
  {
    if (msg.request.size() == 0) {
      out << "request: []";
    } else {
      out << "request: [";
      size_t pending_items = msg.request.size();
      for (auto item : msg.request) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: response
  {
    if (msg.response.size() == 0) {
      out << "response: []";
    } else {
      out << "response: [";
      size_t pending_items = msg.response.size();
      for (auto item : msg.response) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const RequestControlParams_Event & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: info
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "info:\n";
    to_block_style_yaml(msg.info, out, indentation + 2);
  }

  // member: request
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.request.size() == 0) {
      out << "request: []\n";
    } else {
      out << "request:\n";
      for (auto item : msg.request) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: response
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.response.size() == 0) {
      out << "response: []\n";
    } else {
      out << "response:\n";
      for (auto item : msg.response) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const RequestControlParams_Event & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace torsobot_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use torsobot_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const torsobot_interfaces::srv::RequestControlParams_Event & msg,
  std::ostream & out, size_t indentation = 0)
{
  torsobot_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use torsobot_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const torsobot_interfaces::srv::RequestControlParams_Event & msg)
{
  return torsobot_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<torsobot_interfaces::srv::RequestControlParams_Event>()
{
  return "torsobot_interfaces::srv::RequestControlParams_Event";
}

template<>
inline const char * name<torsobot_interfaces::srv::RequestControlParams_Event>()
{
  return "torsobot_interfaces/srv/RequestControlParams_Event";
}

template<>
struct has_fixed_size<torsobot_interfaces::srv::RequestControlParams_Event>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<torsobot_interfaces::srv::RequestControlParams_Event>
  : std::integral_constant<bool, has_bounded_size<service_msgs::msg::ServiceEventInfo>::value && has_bounded_size<torsobot_interfaces::srv::RequestControlParams_Request>::value && has_bounded_size<torsobot_interfaces::srv::RequestControlParams_Response>::value> {};

template<>
struct is_message<torsobot_interfaces::srv::RequestControlParams_Event>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<torsobot_interfaces::srv::RequestControlParams>()
{
  return "torsobot_interfaces::srv::RequestControlParams";
}

template<>
inline const char * name<torsobot_interfaces::srv::RequestControlParams>()
{
  return "torsobot_interfaces/srv/RequestControlParams";
}

template<>
struct has_fixed_size<torsobot_interfaces::srv::RequestControlParams>
  : std::integral_constant<
    bool,
    has_fixed_size<torsobot_interfaces::srv::RequestControlParams_Request>::value &&
    has_fixed_size<torsobot_interfaces::srv::RequestControlParams_Response>::value
  >
{
};

template<>
struct has_bounded_size<torsobot_interfaces::srv::RequestControlParams>
  : std::integral_constant<
    bool,
    has_bounded_size<torsobot_interfaces::srv::RequestControlParams_Request>::value &&
    has_bounded_size<torsobot_interfaces::srv::RequestControlParams_Response>::value
  >
{
};

template<>
struct is_service<torsobot_interfaces::srv::RequestControlParams>
  : std::true_type
{
};

template<>
struct is_service_request<torsobot_interfaces::srv::RequestControlParams_Request>
  : std::true_type
{
};

template<>
struct is_service_response<torsobot_interfaces::srv::RequestControlParams_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // TORSOBOT_INTERFACES__SRV__DETAIL__REQUEST_CONTROL_PARAMS__TRAITS_HPP_
