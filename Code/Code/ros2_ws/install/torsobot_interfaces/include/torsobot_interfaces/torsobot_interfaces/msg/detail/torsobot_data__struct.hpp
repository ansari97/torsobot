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


// Include directives for member types
// Member 'torsobot_state'
#include "torsobot_interfaces/msg/detail/torsobot_state__struct.hpp"

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
  : torsobot_state(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->wheel_torque = 0.0;
      this->wheel_cmd_torque = 0.0;
      this->mot_drv_mode = 0;
      this->mot_pos = 0.0;
      this->mot_vel = 0.0;
      this->mot_pos_init = 0.0;
      this->torso_pitch_init = 0.0;
    }
  }

  explicit TorsobotData_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : torsobot_state(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->wheel_torque = 0.0;
      this->wheel_cmd_torque = 0.0;
      this->mot_drv_mode = 0;
      this->mot_pos = 0.0;
      this->mot_vel = 0.0;
      this->mot_pos_init = 0.0;
      this->torso_pitch_init = 0.0;
    }
  }

  // field types and members
  using _torsobot_state_type =
    torsobot_interfaces::msg::TorsobotState_<ContainerAllocator>;
  _torsobot_state_type torsobot_state;
  using _wheel_torque_type =
    double;
  _wheel_torque_type wheel_torque;
  using _wheel_cmd_torque_type =
    double;
  _wheel_cmd_torque_type wheel_cmd_torque;
  using _mot_drv_mode_type =
    int8_t;
  _mot_drv_mode_type mot_drv_mode;
  using _mot_pos_type =
    double;
  _mot_pos_type mot_pos;
  using _mot_vel_type =
    double;
  _mot_vel_type mot_vel;
  using _mot_pos_init_type =
    double;
  _mot_pos_init_type mot_pos_init;
  using _torso_pitch_init_type =
    double;
  _torso_pitch_init_type torso_pitch_init;

  // setters for named parameter idiom
  Type & set__torsobot_state(
    const torsobot_interfaces::msg::TorsobotState_<ContainerAllocator> & _arg)
  {
    this->torsobot_state = _arg;
    return *this;
  }
  Type & set__wheel_torque(
    const double & _arg)
  {
    this->wheel_torque = _arg;
    return *this;
  }
  Type & set__wheel_cmd_torque(
    const double & _arg)
  {
    this->wheel_cmd_torque = _arg;
    return *this;
  }
  Type & set__mot_drv_mode(
    const int8_t & _arg)
  {
    this->mot_drv_mode = _arg;
    return *this;
  }
  Type & set__mot_pos(
    const double & _arg)
  {
    this->mot_pos = _arg;
    return *this;
  }
  Type & set__mot_vel(
    const double & _arg)
  {
    this->mot_vel = _arg;
    return *this;
  }
  Type & set__mot_pos_init(
    const double & _arg)
  {
    this->mot_pos_init = _arg;
    return *this;
  }
  Type & set__torso_pitch_init(
    const double & _arg)
  {
    this->torso_pitch_init = _arg;
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
    if (this->torsobot_state != other.torsobot_state) {
      return false;
    }
    if (this->wheel_torque != other.wheel_torque) {
      return false;
    }
    if (this->wheel_cmd_torque != other.wheel_cmd_torque) {
      return false;
    }
    if (this->mot_drv_mode != other.mot_drv_mode) {
      return false;
    }
    if (this->mot_pos != other.mot_pos) {
      return false;
    }
    if (this->mot_vel != other.mot_vel) {
      return false;
    }
    if (this->mot_pos_init != other.mot_pos_init) {
      return false;
    }
    if (this->torso_pitch_init != other.torso_pitch_init) {
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
