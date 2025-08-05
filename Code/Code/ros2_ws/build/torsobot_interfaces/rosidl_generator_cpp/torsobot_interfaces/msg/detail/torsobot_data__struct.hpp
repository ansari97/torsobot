// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from torsobot_interfaces:msg/TorsobotData.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "torsobot_interfaces/msg/torsobot_data.hpp"


#ifndef TORSOBOT_INTERFACES__MSG__DETAIL__TORSOBOT_DATA__STRUCT_HPP_
#define TORSOBOT_INTERFACES__MSG__DETAIL__TORSOBOT_DATA__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__torsobot_interfaces__msg__TorsobotData __attribute__((deprecated))
#else
# define DEPRECATED__torsobot_interfaces__msg__TorsobotData __declspec(deprecated)
#endif

namespace torsobot_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct TorsobotData_
{
  using Type = TorsobotData_<ContainerAllocator>;

  explicit TorsobotData_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->torso_pitch = 0.0;
      this->torso_pitch_rate = 0.0;
      this->motor_pos = 0.0;
      this->motor_vel = 0.0;
      this->motor_torque = 0.0;
      this->motor_cmd_torque = 0.0;
      this->motor_drv_mode = 0;
    }
  }

  explicit TorsobotData_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->torso_pitch = 0.0;
      this->torso_pitch_rate = 0.0;
      this->motor_pos = 0.0;
      this->motor_vel = 0.0;
      this->motor_torque = 0.0;
      this->motor_cmd_torque = 0.0;
      this->motor_drv_mode = 0;
    }
  }

  // field types and members
  using _torso_pitch_type =
    double;
  _torso_pitch_type torso_pitch;
  using _torso_pitch_rate_type =
    double;
  _torso_pitch_rate_type torso_pitch_rate;
  using _motor_pos_type =
    double;
  _motor_pos_type motor_pos;
  using _motor_vel_type =
    double;
  _motor_vel_type motor_vel;
  using _motor_torque_type =
    double;
  _motor_torque_type motor_torque;
  using _motor_cmd_torque_type =
    double;
  _motor_cmd_torque_type motor_cmd_torque;
  using _motor_drv_mode_type =
    int8_t;
  _motor_drv_mode_type motor_drv_mode;

  // setters for named parameter idiom
  Type & set__torso_pitch(
    const double & _arg)
  {
    this->torso_pitch = _arg;
    return *this;
  }
  Type & set__torso_pitch_rate(
    const double & _arg)
  {
    this->torso_pitch_rate = _arg;
    return *this;
  }
  Type & set__motor_pos(
    const double & _arg)
  {
    this->motor_pos = _arg;
    return *this;
  }
  Type & set__motor_vel(
    const double & _arg)
  {
    this->motor_vel = _arg;
    return *this;
  }
  Type & set__motor_torque(
    const double & _arg)
  {
    this->motor_torque = _arg;
    return *this;
  }
  Type & set__motor_cmd_torque(
    const double & _arg)
  {
    this->motor_cmd_torque = _arg;
    return *this;
  }
  Type & set__motor_drv_mode(
    const int8_t & _arg)
  {
    this->motor_drv_mode = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    torsobot_interfaces::msg::TorsobotData_<ContainerAllocator> *;
  using ConstRawPtr =
    const torsobot_interfaces::msg::TorsobotData_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<torsobot_interfaces::msg::TorsobotData_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<torsobot_interfaces::msg::TorsobotData_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      torsobot_interfaces::msg::TorsobotData_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<torsobot_interfaces::msg::TorsobotData_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      torsobot_interfaces::msg::TorsobotData_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<torsobot_interfaces::msg::TorsobotData_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<torsobot_interfaces::msg::TorsobotData_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<torsobot_interfaces::msg::TorsobotData_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__torsobot_interfaces__msg__TorsobotData
    std::shared_ptr<torsobot_interfaces::msg::TorsobotData_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__torsobot_interfaces__msg__TorsobotData
    std::shared_ptr<torsobot_interfaces::msg::TorsobotData_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const TorsobotData_ & other) const
  {
    if (this->torso_pitch != other.torso_pitch) {
      return false;
    }
    if (this->torso_pitch_rate != other.torso_pitch_rate) {
      return false;
    }
    if (this->motor_pos != other.motor_pos) {
      return false;
    }
    if (this->motor_vel != other.motor_vel) {
      return false;
    }
    if (this->motor_torque != other.motor_torque) {
      return false;
    }
    if (this->motor_cmd_torque != other.motor_cmd_torque) {
      return false;
    }
    if (this->motor_drv_mode != other.motor_drv_mode) {
      return false;
    }
    return true;
  }
  bool operator!=(const TorsobotData_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct TorsobotData_

// alias to use template instance with default allocator
using TorsobotData =
  torsobot_interfaces::msg::TorsobotData_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace torsobot_interfaces

#endif  // TORSOBOT_INTERFACES__MSG__DETAIL__TORSOBOT_DATA__STRUCT_HPP_
