// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rc110_msgs:msg/BaseboardError.idl
// generated code does not contain a copyright notice

#ifndef RC110_MSGS__MSG__DETAIL__BASEBOARD_ERROR__BUILDER_HPP_
#define RC110_MSGS__MSG__DETAIL__BASEBOARD_ERROR__BUILDER_HPP_

#include "rc110_msgs/msg/detail/baseboard_error__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace rc110_msgs
{

namespace msg
{

namespace builder
{

class Init_BaseboardError_data
{
public:
  Init_BaseboardError_data()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::rc110_msgs::msg::BaseboardError data(::rc110_msgs::msg::BaseboardError::_data_type arg)
  {
    msg_.data = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rc110_msgs::msg::BaseboardError msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::rc110_msgs::msg::BaseboardError>()
{
  return rc110_msgs::msg::builder::Init_BaseboardError_data();
}

}  // namespace rc110_msgs

#endif  // RC110_MSGS__MSG__DETAIL__BASEBOARD_ERROR__BUILDER_HPP_
