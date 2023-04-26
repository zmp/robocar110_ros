// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rc110_msgs:msg/StringArray.idl
// generated code does not contain a copyright notice

#ifndef RC110_MSGS__MSG__DETAIL__STRING_ARRAY__BUILDER_HPP_
#define RC110_MSGS__MSG__DETAIL__STRING_ARRAY__BUILDER_HPP_

#include "rc110_msgs/msg/detail/string_array__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace rc110_msgs
{

namespace msg
{

namespace builder
{

class Init_StringArray_data
{
public:
  explicit Init_StringArray_data(::rc110_msgs::msg::StringArray & msg)
  : msg_(msg)
  {}
  ::rc110_msgs::msg::StringArray data(::rc110_msgs::msg::StringArray::_data_type arg)
  {
    msg_.data = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rc110_msgs::msg::StringArray msg_;
};

class Init_StringArray_header
{
public:
  Init_StringArray_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_StringArray_data header(::rc110_msgs::msg::StringArray::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_StringArray_data(msg_);
  }

private:
  ::rc110_msgs::msg::StringArray msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::rc110_msgs::msg::StringArray>()
{
  return rc110_msgs::msg::builder::Init_StringArray_header();
}

}  // namespace rc110_msgs

#endif  // RC110_MSGS__MSG__DETAIL__STRING_ARRAY__BUILDER_HPP_
