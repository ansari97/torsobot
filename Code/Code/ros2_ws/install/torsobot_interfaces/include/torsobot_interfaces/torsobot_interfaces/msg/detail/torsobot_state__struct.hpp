// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from torsobot_interfaces:msg/TorsobotState.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "torsobot_interfaces/msg/torsobot_state.hpp"


#ifndef TORSOBOT_INTERFACES__MSG__DETAIL__TORSOBOT_STATE__STRUCT_HPP_
#define TORSOBOT_INTERFACES__MSG__DETAIL__TORSOBOT_STATE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__torsobot_interfaces__msg__TorsobotState __attribute__((deprecated))
#else
# define DEPRECATED__torsobot_interfaces__msg__TorsobotState __declspec(deprecated)
#endif

namespace torsobot_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct TorsobotState_
{
  using Type = TorsobotState_<ContainerAllocator>;

  explicit TorsobotState_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->torso_pitch = 0.0;
      this->torso_pitch_rate = 0.0;
      this->wheel_pos = 0.0;
      this->wheel_vel = 0.0;
    }
  }

  explicit TorsobotState_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->torso_pitch = 0.0;
      this->torso_pitch_rate = 0.0;
      this->wheel_pos = 0.0;
      this->wheel_vel = 0.0;
    }
  }

  // field types and members
  using _torso_pitch_type =
    double;
  _torso_pitch_type torso_pitch;
  using _torso_pitch_rate_type =
    double;
  _torso_pitch_rate_type torso_pitch_rate;
  using _wheel_pos_type =
    double;
  _wheel_pos_type wheel_pos;
  using _wheel_vel_type =
    double;
  _wheel_vel_type wheel_vel;

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
  Type & set__wheel_pos(
    const double & _arg)
  {
    this->wheel_pos = _arg;
    return *this;
  }
  Type & set__wheel_vel(
    const double & _arg)
  {
    this->wheel_vel = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    torsobot_interfaces::msg::TorsobotState_<ContainerAllocator> *;
  using ConstRawPtr =
    const torsobot_interfaces::msg::TorsobotState_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<torsobot_interfaces::msg::TorsobotState_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<torsobot_interfaces::msg::TorsobotState_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      torsobot_interfaces::msg::TorsobotState_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<torsobot_interfaces::msg::TorsobotState_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      torsobot_interfaces::msg::TorsobotState_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<torsobot_interfaces::msg::TorsobotState_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<torsobot_interfaces::msg::TorsobotState_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<torsobot_interfaces::msg::TorsobotState_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__torsobot_interfaces__msg__TorsobotState
    std::shared_ptr<torsobot_interfaces::msg::TorsobotState_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__torsobot_interfaces__msg__TorsobotState
    std::shared_ptr<torsobot_interfaces::msg::TorsobotState_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const TorsobotState_ & other) const
  {
    if (this->torso_pitch != other.torso_pitch) {
      return false;
    }
    if (this->torso_pitch_rate != other.torso_pitch_rate) {
      return false;
    }
    if (this->wheel_pos != other.wheel_pos) {
      return false;
    }
    if (this->wheel_vel != other.wheel_vel) {
      return false;
    }
    return true;
  }
  bool operator!=(const TorsobotState_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct TorsobotState_

// alias to use template instance with default allocator
using TorsobotState =
  torsobot_interfaces::msg::TorsobotState_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace torsobot_interfaces

#endif  // TORSOBOT_INTERFACES__MSG__DETAIL__TORSOBOT_STATE__STRUCT_HPP_
