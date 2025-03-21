// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from torsobot_interfaces:msg/Data.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "torsobot_interfaces/msg/data.hpp"


#ifndef TORSOBOT_INTERFACES__MSG__DETAIL__DATA__STRUCT_HPP_
#define TORSOBOT_INTERFACES__MSG__DETAIL__DATA__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__torsobot_interfaces__msg__Data __attribute__((deprecated))
#else
# define DEPRECATED__torsobot_interfaces__msg__Data __declspec(deprecated)
#endif

namespace torsobot_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Data_
{
  using Type = Data_<ContainerAllocator>;

  explicit Data_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->torso_pitch = 0.0;
      this->wheel_pitch = 0.0;
      this->motor_current = 0.0;
    }
  }

  explicit Data_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->torso_pitch = 0.0;
      this->wheel_pitch = 0.0;
      this->motor_current = 0.0;
    }
  }

  // field types and members
  using _torso_pitch_type =
    double;
  _torso_pitch_type torso_pitch;
  using _wheel_pitch_type =
    double;
  _wheel_pitch_type wheel_pitch;
  using _motor_current_type =
    double;
  _motor_current_type motor_current;

  // setters for named parameter idiom
  Type & set__torso_pitch(
    const double & _arg)
  {
    this->torso_pitch = _arg;
    return *this;
  }
  Type & set__wheel_pitch(
    const double & _arg)
  {
    this->wheel_pitch = _arg;
    return *this;
  }
  Type & set__motor_current(
    const double & _arg)
  {
    this->motor_current = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    torsobot_interfaces::msg::Data_<ContainerAllocator> *;
  using ConstRawPtr =
    const torsobot_interfaces::msg::Data_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<torsobot_interfaces::msg::Data_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<torsobot_interfaces::msg::Data_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      torsobot_interfaces::msg::Data_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<torsobot_interfaces::msg::Data_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      torsobot_interfaces::msg::Data_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<torsobot_interfaces::msg::Data_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<torsobot_interfaces::msg::Data_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<torsobot_interfaces::msg::Data_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__torsobot_interfaces__msg__Data
    std::shared_ptr<torsobot_interfaces::msg::Data_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__torsobot_interfaces__msg__Data
    std::shared_ptr<torsobot_interfaces::msg::Data_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Data_ & other) const
  {
    if (this->torso_pitch != other.torso_pitch) {
      return false;
    }
    if (this->wheel_pitch != other.wheel_pitch) {
      return false;
    }
    if (this->motor_current != other.motor_current) {
      return false;
    }
    return true;
  }
  bool operator!=(const Data_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Data_

// alias to use template instance with default allocator
using Data =
  torsobot_interfaces::msg::Data_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace torsobot_interfaces

#endif  // TORSOBOT_INTERFACES__MSG__DETAIL__DATA__STRUCT_HPP_
