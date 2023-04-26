// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from rc110_msgs:msg/Offsets.idl
// generated code does not contain a copyright notice

#ifndef RC110_MSGS__MSG__DETAIL__OFFSETS__STRUCT_HPP_
#define RC110_MSGS__MSG__DETAIL__OFFSETS__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__rc110_msgs__msg__Offsets __attribute__((deprecated))
#else
# define DEPRECATED__rc110_msgs__msg__Offsets __declspec(deprecated)
#endif

namespace rc110_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Offsets_
{
  using Type = Offsets_<ContainerAllocator>;

  explicit Offsets_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->gyro = 0.0f;
      this->accel_x = 0.0f;
      this->accel_y = 0.0f;
      this->accel_z = 0.0f;
      this->motor_current = 0.0f;
      this->steering = 0.0f;
    }
  }

  explicit Offsets_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->gyro = 0.0f;
      this->accel_x = 0.0f;
      this->accel_y = 0.0f;
      this->accel_z = 0.0f;
      this->motor_current = 0.0f;
      this->steering = 0.0f;
    }
  }

  // field types and members
  using _gyro_type =
    float;
  _gyro_type gyro;
  using _accel_x_type =
    float;
  _accel_x_type accel_x;
  using _accel_y_type =
    float;
  _accel_y_type accel_y;
  using _accel_z_type =
    float;
  _accel_z_type accel_z;
  using _motor_current_type =
    float;
  _motor_current_type motor_current;
  using _steering_type =
    float;
  _steering_type steering;

  // setters for named parameter idiom
  Type & set__gyro(
    const float & _arg)
  {
    this->gyro = _arg;
    return *this;
  }
  Type & set__accel_x(
    const float & _arg)
  {
    this->accel_x = _arg;
    return *this;
  }
  Type & set__accel_y(
    const float & _arg)
  {
    this->accel_y = _arg;
    return *this;
  }
  Type & set__accel_z(
    const float & _arg)
  {
    this->accel_z = _arg;
    return *this;
  }
  Type & set__motor_current(
    const float & _arg)
  {
    this->motor_current = _arg;
    return *this;
  }
  Type & set__steering(
    const float & _arg)
  {
    this->steering = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    rc110_msgs::msg::Offsets_<ContainerAllocator> *;
  using ConstRawPtr =
    const rc110_msgs::msg::Offsets_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rc110_msgs::msg::Offsets_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rc110_msgs::msg::Offsets_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rc110_msgs::msg::Offsets_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rc110_msgs::msg::Offsets_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rc110_msgs::msg::Offsets_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rc110_msgs::msg::Offsets_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rc110_msgs::msg::Offsets_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rc110_msgs::msg::Offsets_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rc110_msgs__msg__Offsets
    std::shared_ptr<rc110_msgs::msg::Offsets_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rc110_msgs__msg__Offsets
    std::shared_ptr<rc110_msgs::msg::Offsets_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Offsets_ & other) const
  {
    if (this->gyro != other.gyro) {
      return false;
    }
    if (this->accel_x != other.accel_x) {
      return false;
    }
    if (this->accel_y != other.accel_y) {
      return false;
    }
    if (this->accel_z != other.accel_z) {
      return false;
    }
    if (this->motor_current != other.motor_current) {
      return false;
    }
    if (this->steering != other.steering) {
      return false;
    }
    return true;
  }
  bool operator!=(const Offsets_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Offsets_

// alias to use template instance with default allocator
using Offsets =
  rc110_msgs::msg::Offsets_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace rc110_msgs

#endif  // RC110_MSGS__MSG__DETAIL__OFFSETS__STRUCT_HPP_
