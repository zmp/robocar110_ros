// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from rc110_msgs:msg/StringArray.idl
// generated code does not contain a copyright notice

#ifndef RC110_MSGS__MSG__DETAIL__STRING_ARRAY__STRUCT_HPP_
#define RC110_MSGS__MSG__DETAIL__STRING_ARRAY__STRUCT_HPP_

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
# define DEPRECATED__rc110_msgs__msg__StringArray __attribute__((deprecated))
#else
# define DEPRECATED__rc110_msgs__msg__StringArray __declspec(deprecated)
#endif

namespace rc110_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct StringArray_
{
  using Type = StringArray_<ContainerAllocator>;

  explicit StringArray_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    (void)_init;
  }

  explicit StringArray_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _data_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>, typename ContainerAllocator::template rebind<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>>::other>;
  _data_type data;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__data(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>, typename ContainerAllocator::template rebind<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>>::other> & _arg)
  {
    this->data = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    rc110_msgs::msg::StringArray_<ContainerAllocator> *;
  using ConstRawPtr =
    const rc110_msgs::msg::StringArray_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rc110_msgs::msg::StringArray_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rc110_msgs::msg::StringArray_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rc110_msgs::msg::StringArray_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rc110_msgs::msg::StringArray_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rc110_msgs::msg::StringArray_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rc110_msgs::msg::StringArray_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rc110_msgs::msg::StringArray_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rc110_msgs::msg::StringArray_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rc110_msgs__msg__StringArray
    std::shared_ptr<rc110_msgs::msg::StringArray_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rc110_msgs__msg__StringArray
    std::shared_ptr<rc110_msgs::msg::StringArray_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const StringArray_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->data != other.data) {
      return false;
    }
    return true;
  }
  bool operator!=(const StringArray_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct StringArray_

// alias to use template instance with default allocator
using StringArray =
  rc110_msgs::msg::StringArray_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace rc110_msgs

#endif  // RC110_MSGS__MSG__DETAIL__STRING_ARRAY__STRUCT_HPP_
