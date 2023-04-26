// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rc110_msgs:srv/SetInteger.idl
// generated code does not contain a copyright notice

#ifndef RC110_MSGS__SRV__DETAIL__SET_INTEGER__BUILDER_HPP_
#define RC110_MSGS__SRV__DETAIL__SET_INTEGER__BUILDER_HPP_

#include "rc110_msgs/srv/detail/set_integer__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace rc110_msgs
{

namespace srv
{

namespace builder
{

class Init_SetInteger_Request_data
{
public:
  Init_SetInteger_Request_data()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::rc110_msgs::srv::SetInteger_Request data(::rc110_msgs::srv::SetInteger_Request::_data_type arg)
  {
    msg_.data = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rc110_msgs::srv::SetInteger_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::rc110_msgs::srv::SetInteger_Request>()
{
  return rc110_msgs::srv::builder::Init_SetInteger_Request_data();
}

}  // namespace rc110_msgs


namespace rc110_msgs
{

namespace srv
{

namespace builder
{

class Init_SetInteger_Response_message
{
public:
  explicit Init_SetInteger_Response_message(::rc110_msgs::srv::SetInteger_Response & msg)
  : msg_(msg)
  {}
  ::rc110_msgs::srv::SetInteger_Response message(::rc110_msgs::srv::SetInteger_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rc110_msgs::srv::SetInteger_Response msg_;
};

class Init_SetInteger_Response_success
{
public:
  Init_SetInteger_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetInteger_Response_message success(::rc110_msgs::srv::SetInteger_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_SetInteger_Response_message(msg_);
  }

private:
  ::rc110_msgs::srv::SetInteger_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::rc110_msgs::srv::SetInteger_Response>()
{
  return rc110_msgs::srv::builder::Init_SetInteger_Response_success();
}

}  // namespace rc110_msgs

#endif  // RC110_MSGS__SRV__DETAIL__SET_INTEGER__BUILDER_HPP_
