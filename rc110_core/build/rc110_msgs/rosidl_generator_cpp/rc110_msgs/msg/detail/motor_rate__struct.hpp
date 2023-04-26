// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from rc110_msgs:msg/MotorRate.idl
// generated code does not contain a copyright notice

#ifndef RC110_MSGS__MSG__DETAIL__MOTOR_RATE__STRUCT_HPP_
#define RC110_MSGS__MSG__DETAIL__MOTOR_RATE__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__rc110_msgs__msg__MotorRate __attribute__((deprecated))
#else
# define DEPRECATED__rc110_msgs__msg__MotorRate __declspec(deprecated)
#endif

namespace rc110_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MotorRate_
{
  using Type = MotorRate_<ContainerAllocator>;

  explicit MotorRate_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->motor_rate = 0.0f;
      this->estimated_speed = 0.0f;
    }
  }

  explicit MotorRate_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->motor_rate = 0.0f;
      this->estimated_speed = 0.0f;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _motor_rate_type =
    float;
  _motor_rate_type motor_rate;
  using _estimated_speed_type =
    float;
  _estimated_speed_type estimated_speed;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__motor_rate(
    const float & _arg)
  {
    this->motor_rate = _arg;
    return *this;
  }
  Type & set__estimated_speed(
    const float & _arg)
  {
    this->estimated_speed = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    rc110_msgs::msg::MotorRate_<ContainerAllocator> *;
  using ConstRawPtr =
    const rc110_msgs::msg::MotorRate_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rc110_msgs::msg::MotorRate_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rc110_msgs::msg::MotorRate_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rc110_msgs::msg::MotorRate_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rc110_msgs::msg::MotorRate_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rc110_msgs::msg::MotorRate_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rc110_msgs::msg::MotorRate_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rc110_msgs::msg::MotorRate_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rc110_msgs::msg::MotorRate_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rc110_msgs__msg__MotorRate
    std::shared_ptr<rc110_msgs::msg::MotorRate_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rc110_msgs__msg__MotorRate
    std::shared_ptr<rc110_msgs::msg::MotorRate_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MotorRate_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->motor_rate != other.motor_rate) {
      return false;
    }
    if (this->estimated_speed != other.estimated_speed) {
      return false;
    }
    return true;
  }
  bool operator!=(const MotorRate_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MotorRate_

// alias to use template instance with default allocator
using MotorRate =
  rc110_msgs::msg::MotorRate_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace rc110_msgs

#endif  // RC110_MSGS__MSG__DETAIL__MOTOR_RATE__STRUCT_HPP_
