// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from rc110_msgs:srv/SetInteger.idl
// generated code does not contain a copyright notice

#ifndef RC110_MSGS__SRV__DETAIL__SET_INTEGER__STRUCT_HPP_
#define RC110_MSGS__SRV__DETAIL__SET_INTEGER__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__rc110_msgs__srv__SetInteger_Request __attribute__((deprecated))
#else
# define DEPRECATED__rc110_msgs__srv__SetInteger_Request __declspec(deprecated)
#endif

namespace rc110_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SetInteger_Request_
{
  using Type = SetInteger_Request_<ContainerAllocator>;

  explicit SetInteger_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->data = 0l;
    }
  }

  explicit SetInteger_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->data = 0l;
    }
  }

  // field types and members
  using _data_type =
    int32_t;
  _data_type data;

  // setters for named parameter idiom
  Type & set__data(
    const int32_t & _arg)
  {
    this->data = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    rc110_msgs::srv::SetInteger_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const rc110_msgs::srv::SetInteger_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rc110_msgs::srv::SetInteger_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rc110_msgs::srv::SetInteger_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rc110_msgs::srv::SetInteger_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rc110_msgs::srv::SetInteger_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rc110_msgs::srv::SetInteger_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rc110_msgs::srv::SetInteger_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rc110_msgs::srv::SetInteger_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rc110_msgs::srv::SetInteger_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rc110_msgs__srv__SetInteger_Request
    std::shared_ptr<rc110_msgs::srv::SetInteger_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rc110_msgs__srv__SetInteger_Request
    std::shared_ptr<rc110_msgs::srv::SetInteger_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SetInteger_Request_ & other) const
  {
    if (this->data != other.data) {
      return false;
    }
    return true;
  }
  bool operator!=(const SetInteger_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SetInteger_Request_

// alias to use template instance with default allocator
using SetInteger_Request =
  rc110_msgs::srv::SetInteger_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace rc110_msgs


#ifndef _WIN32
# define DEPRECATED__rc110_msgs__srv__SetInteger_Response __attribute__((deprecated))
#else
# define DEPRECATED__rc110_msgs__srv__SetInteger_Response __declspec(deprecated)
#endif

namespace rc110_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SetInteger_Response_
{
  using Type = SetInteger_Response_<ContainerAllocator>;

  explicit SetInteger_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  explicit SetInteger_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : message(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;
  using _message_type =
    std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>;
  _message_type message;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }
  Type & set__message(
    const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other> & _arg)
  {
    this->message = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    rc110_msgs::srv::SetInteger_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const rc110_msgs::srv::SetInteger_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rc110_msgs::srv::SetInteger_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rc110_msgs::srv::SetInteger_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rc110_msgs::srv::SetInteger_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rc110_msgs::srv::SetInteger_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rc110_msgs::srv::SetInteger_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rc110_msgs::srv::SetInteger_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rc110_msgs::srv::SetInteger_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rc110_msgs::srv::SetInteger_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rc110_msgs__srv__SetInteger_Response
    std::shared_ptr<rc110_msgs::srv::SetInteger_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rc110_msgs__srv__SetInteger_Response
    std::shared_ptr<rc110_msgs::srv::SetInteger_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SetInteger_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->message != other.message) {
      return false;
    }
    return true;
  }
  bool operator!=(const SetInteger_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SetInteger_Response_

// alias to use template instance with default allocator
using SetInteger_Response =
  rc110_msgs::srv::SetInteger_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace rc110_msgs

namespace rc110_msgs
{

namespace srv
{

struct SetInteger
{
  using Request = rc110_msgs::srv::SetInteger_Request;
  using Response = rc110_msgs::srv::SetInteger_Response;
};

}  // namespace srv

}  // namespace rc110_msgs

#endif  // RC110_MSGS__SRV__DETAIL__SET_INTEGER__STRUCT_HPP_
