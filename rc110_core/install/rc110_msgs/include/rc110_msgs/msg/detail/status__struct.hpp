// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from rc110_msgs:msg/Status.idl
// generated code does not contain a copyright notice

#ifndef RC110_MSGS__MSG__DETAIL__STATUS__STRUCT_HPP_
#define RC110_MSGS__MSG__DETAIL__STATUS__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__rc110_msgs__msg__Status __attribute__((deprecated))
#else
# define DEPRECATED__rc110_msgs__msg__Status __declspec(deprecated)
#endif

namespace rc110_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Status_
{
  using Type = Status_<ContainerAllocator>;

  explicit Status_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->board_enabled = false;
      this->motor_state = 0;
      this->servo_state = 0;
    }
  }

  explicit Status_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->board_enabled = false;
      this->motor_state = 0;
      this->servo_state = 0;
    }
  }

  // field types and members
  using _board_enabled_type =
    bool;
  _board_enabled_type board_enabled;
  using _motor_state_type =
    uint8_t;
  _motor_state_type motor_state;
  using _servo_state_type =
    uint8_t;
  _servo_state_type servo_state;

  // setters for named parameter idiom
  Type & set__board_enabled(
    const bool & _arg)
  {
    this->board_enabled = _arg;
    return *this;
  }
  Type & set__motor_state(
    const uint8_t & _arg)
  {
    this->motor_state = _arg;
    return *this;
  }
  Type & set__servo_state(
    const uint8_t & _arg)
  {
    this->servo_state = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t MOTOR_OFF =
    0u;
  static constexpr uint8_t MOTOR_ON =
    1u;
  static constexpr uint8_t MOTOR_NEUTRAL =
    2u;

  // pointer types
  using RawPtr =
    rc110_msgs::msg::Status_<ContainerAllocator> *;
  using ConstRawPtr =
    const rc110_msgs::msg::Status_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rc110_msgs::msg::Status_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rc110_msgs::msg::Status_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rc110_msgs::msg::Status_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rc110_msgs::msg::Status_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rc110_msgs::msg::Status_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rc110_msgs::msg::Status_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rc110_msgs::msg::Status_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rc110_msgs::msg::Status_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rc110_msgs__msg__Status
    std::shared_ptr<rc110_msgs::msg::Status_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rc110_msgs__msg__Status
    std::shared_ptr<rc110_msgs::msg::Status_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Status_ & other) const
  {
    if (this->board_enabled != other.board_enabled) {
      return false;
    }
    if (this->motor_state != other.motor_state) {
      return false;
    }
    if (this->servo_state != other.servo_state) {
      return false;
    }
    return true;
  }
  bool operator!=(const Status_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Status_

// alias to use template instance with default allocator
using Status =
  rc110_msgs::msg::Status_<std::allocator<void>>;

// constant definitions
template<typename ContainerAllocator>
constexpr uint8_t Status_<ContainerAllocator>::MOTOR_OFF;
template<typename ContainerAllocator>
constexpr uint8_t Status_<ContainerAllocator>::MOTOR_ON;
template<typename ContainerAllocator>
constexpr uint8_t Status_<ContainerAllocator>::MOTOR_NEUTRAL;

}  // namespace msg

}  // namespace rc110_msgs

#endif  // RC110_MSGS__MSG__DETAIL__STATUS__STRUCT_HPP_
