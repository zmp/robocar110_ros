// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from rc110_msgs:msg/BaseboardError.idl
// generated code does not contain a copyright notice

#ifndef RC110_MSGS__MSG__DETAIL__BASEBOARD_ERROR__STRUCT_HPP_
#define RC110_MSGS__MSG__DETAIL__BASEBOARD_ERROR__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__rc110_msgs__msg__BaseboardError __attribute__((deprecated))
#else
# define DEPRECATED__rc110_msgs__msg__BaseboardError __declspec(deprecated)
#endif

namespace rc110_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct BaseboardError_
{
  using Type = BaseboardError_<ContainerAllocator>;

  explicit BaseboardError_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->data = 0;
    }
  }

  explicit BaseboardError_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->data = 0;
    }
  }

  // field types and members
  using _data_type =
    uint8_t;
  _data_type data;

  // setters for named parameter idiom
  Type & set__data(
    const uint8_t & _arg)
  {
    this->data = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t NONE =
    0u;
  static constexpr uint8_t BOARD_HEAT =
    1u;
  static constexpr uint8_t MOTOR_HEAT =
    2u;
  static constexpr uint8_t MOTOR_FAILURE =
    3u;
  static constexpr uint8_t LOW_VOLTAGE =
    4u;

  // pointer types
  using RawPtr =
    rc110_msgs::msg::BaseboardError_<ContainerAllocator> *;
  using ConstRawPtr =
    const rc110_msgs::msg::BaseboardError_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rc110_msgs::msg::BaseboardError_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rc110_msgs::msg::BaseboardError_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rc110_msgs::msg::BaseboardError_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rc110_msgs::msg::BaseboardError_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rc110_msgs::msg::BaseboardError_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rc110_msgs::msg::BaseboardError_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rc110_msgs::msg::BaseboardError_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rc110_msgs::msg::BaseboardError_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rc110_msgs__msg__BaseboardError
    std::shared_ptr<rc110_msgs::msg::BaseboardError_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rc110_msgs__msg__BaseboardError
    std::shared_ptr<rc110_msgs::msg::BaseboardError_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const BaseboardError_ & other) const
  {
    if (this->data != other.data) {
      return false;
    }
    return true;
  }
  bool operator!=(const BaseboardError_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct BaseboardError_

// alias to use template instance with default allocator
using BaseboardError =
  rc110_msgs::msg::BaseboardError_<std::allocator<void>>;

// constant definitions
template<typename ContainerAllocator>
constexpr uint8_t BaseboardError_<ContainerAllocator>::NONE;
template<typename ContainerAllocator>
constexpr uint8_t BaseboardError_<ContainerAllocator>::BOARD_HEAT;
template<typename ContainerAllocator>
constexpr uint8_t BaseboardError_<ContainerAllocator>::MOTOR_HEAT;
template<typename ContainerAllocator>
constexpr uint8_t BaseboardError_<ContainerAllocator>::MOTOR_FAILURE;
template<typename ContainerAllocator>
constexpr uint8_t BaseboardError_<ContainerAllocator>::LOW_VOLTAGE;

}  // namespace msg

}  // namespace rc110_msgs

#endif  // RC110_MSGS__MSG__DETAIL__BASEBOARD_ERROR__STRUCT_HPP_
