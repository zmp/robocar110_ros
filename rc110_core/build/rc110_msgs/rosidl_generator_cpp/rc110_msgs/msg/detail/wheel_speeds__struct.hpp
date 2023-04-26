// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from rc110_msgs:msg/WheelSpeeds.idl
// generated code does not contain a copyright notice

#ifndef RC110_MSGS__MSG__DETAIL__WHEEL_SPEEDS__STRUCT_HPP_
#define RC110_MSGS__MSG__DETAIL__WHEEL_SPEEDS__STRUCT_HPP_

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
# define DEPRECATED__rc110_msgs__msg__WheelSpeeds __attribute__((deprecated))
#else
# define DEPRECATED__rc110_msgs__msg__WheelSpeeds __declspec(deprecated)
#endif

namespace rc110_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct WheelSpeeds_
{
  using Type = WheelSpeeds_<ContainerAllocator>;

  explicit WheelSpeeds_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->speed_fl = 0.0f;
      this->speed_fr = 0.0f;
      this->speed_rl = 0.0f;
      this->speed_rr = 0.0f;
    }
  }

  explicit WheelSpeeds_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->speed_fl = 0.0f;
      this->speed_fr = 0.0f;
      this->speed_rl = 0.0f;
      this->speed_rr = 0.0f;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _speed_fl_type =
    float;
  _speed_fl_type speed_fl;
  using _speed_fr_type =
    float;
  _speed_fr_type speed_fr;
  using _speed_rl_type =
    float;
  _speed_rl_type speed_rl;
  using _speed_rr_type =
    float;
  _speed_rr_type speed_rr;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__speed_fl(
    const float & _arg)
  {
    this->speed_fl = _arg;
    return *this;
  }
  Type & set__speed_fr(
    const float & _arg)
  {
    this->speed_fr = _arg;
    return *this;
  }
  Type & set__speed_rl(
    const float & _arg)
  {
    this->speed_rl = _arg;
    return *this;
  }
  Type & set__speed_rr(
    const float & _arg)
  {
    this->speed_rr = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    rc110_msgs::msg::WheelSpeeds_<ContainerAllocator> *;
  using ConstRawPtr =
    const rc110_msgs::msg::WheelSpeeds_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rc110_msgs::msg::WheelSpeeds_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rc110_msgs::msg::WheelSpeeds_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rc110_msgs::msg::WheelSpeeds_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rc110_msgs::msg::WheelSpeeds_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rc110_msgs::msg::WheelSpeeds_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rc110_msgs::msg::WheelSpeeds_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rc110_msgs::msg::WheelSpeeds_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rc110_msgs::msg::WheelSpeeds_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rc110_msgs__msg__WheelSpeeds
    std::shared_ptr<rc110_msgs::msg::WheelSpeeds_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rc110_msgs__msg__WheelSpeeds
    std::shared_ptr<rc110_msgs::msg::WheelSpeeds_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const WheelSpeeds_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->speed_fl != other.speed_fl) {
      return false;
    }
    if (this->speed_fr != other.speed_fr) {
      return false;
    }
    if (this->speed_rl != other.speed_rl) {
      return false;
    }
    if (this->speed_rr != other.speed_rr) {
      return false;
    }
    return true;
  }
  bool operator!=(const WheelSpeeds_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct WheelSpeeds_

// alias to use template instance with default allocator
using WheelSpeeds =
  rc110_msgs::msg::WheelSpeeds_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace rc110_msgs

#endif  // RC110_MSGS__MSG__DETAIL__WHEEL_SPEEDS__STRUCT_HPP_
