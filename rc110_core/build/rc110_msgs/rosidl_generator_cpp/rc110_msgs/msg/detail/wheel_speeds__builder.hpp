// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rc110_msgs:msg/WheelSpeeds.idl
// generated code does not contain a copyright notice

#ifndef RC110_MSGS__MSG__DETAIL__WHEEL_SPEEDS__BUILDER_HPP_
#define RC110_MSGS__MSG__DETAIL__WHEEL_SPEEDS__BUILDER_HPP_

#include "rc110_msgs/msg/detail/wheel_speeds__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace rc110_msgs
{

namespace msg
{

namespace builder
{

class Init_WheelSpeeds_speed_rr
{
public:
  explicit Init_WheelSpeeds_speed_rr(::rc110_msgs::msg::WheelSpeeds & msg)
  : msg_(msg)
  {}
  ::rc110_msgs::msg::WheelSpeeds speed_rr(::rc110_msgs::msg::WheelSpeeds::_speed_rr_type arg)
  {
    msg_.speed_rr = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rc110_msgs::msg::WheelSpeeds msg_;
};

class Init_WheelSpeeds_speed_rl
{
public:
  explicit Init_WheelSpeeds_speed_rl(::rc110_msgs::msg::WheelSpeeds & msg)
  : msg_(msg)
  {}
  Init_WheelSpeeds_speed_rr speed_rl(::rc110_msgs::msg::WheelSpeeds::_speed_rl_type arg)
  {
    msg_.speed_rl = std::move(arg);
    return Init_WheelSpeeds_speed_rr(msg_);
  }

private:
  ::rc110_msgs::msg::WheelSpeeds msg_;
};

class Init_WheelSpeeds_speed_fr
{
public:
  explicit Init_WheelSpeeds_speed_fr(::rc110_msgs::msg::WheelSpeeds & msg)
  : msg_(msg)
  {}
  Init_WheelSpeeds_speed_rl speed_fr(::rc110_msgs::msg::WheelSpeeds::_speed_fr_type arg)
  {
    msg_.speed_fr = std::move(arg);
    return Init_WheelSpeeds_speed_rl(msg_);
  }

private:
  ::rc110_msgs::msg::WheelSpeeds msg_;
};

class Init_WheelSpeeds_speed_fl
{
public:
  explicit Init_WheelSpeeds_speed_fl(::rc110_msgs::msg::WheelSpeeds & msg)
  : msg_(msg)
  {}
  Init_WheelSpeeds_speed_fr speed_fl(::rc110_msgs::msg::WheelSpeeds::_speed_fl_type arg)
  {
    msg_.speed_fl = std::move(arg);
    return Init_WheelSpeeds_speed_fr(msg_);
  }

private:
  ::rc110_msgs::msg::WheelSpeeds msg_;
};

class Init_WheelSpeeds_header
{
public:
  Init_WheelSpeeds_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_WheelSpeeds_speed_fl header(::rc110_msgs::msg::WheelSpeeds::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_WheelSpeeds_speed_fl(msg_);
  }

private:
  ::rc110_msgs::msg::WheelSpeeds msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::rc110_msgs::msg::WheelSpeeds>()
{
  return rc110_msgs::msg::builder::Init_WheelSpeeds_header();
}

}  // namespace rc110_msgs

#endif  // RC110_MSGS__MSG__DETAIL__WHEEL_SPEEDS__BUILDER_HPP_
