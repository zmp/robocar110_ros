// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rc110_msgs:msg/MotorRate.idl
// generated code does not contain a copyright notice

#ifndef RC110_MSGS__MSG__DETAIL__MOTOR_RATE__BUILDER_HPP_
#define RC110_MSGS__MSG__DETAIL__MOTOR_RATE__BUILDER_HPP_

#include "rc110_msgs/msg/detail/motor_rate__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace rc110_msgs
{

namespace msg
{

namespace builder
{

class Init_MotorRate_estimated_speed
{
public:
  explicit Init_MotorRate_estimated_speed(::rc110_msgs::msg::MotorRate & msg)
  : msg_(msg)
  {}
  ::rc110_msgs::msg::MotorRate estimated_speed(::rc110_msgs::msg::MotorRate::_estimated_speed_type arg)
  {
    msg_.estimated_speed = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rc110_msgs::msg::MotorRate msg_;
};

class Init_MotorRate_motor_rate
{
public:
  explicit Init_MotorRate_motor_rate(::rc110_msgs::msg::MotorRate & msg)
  : msg_(msg)
  {}
  Init_MotorRate_estimated_speed motor_rate(::rc110_msgs::msg::MotorRate::_motor_rate_type arg)
  {
    msg_.motor_rate = std::move(arg);
    return Init_MotorRate_estimated_speed(msg_);
  }

private:
  ::rc110_msgs::msg::MotorRate msg_;
};

class Init_MotorRate_header
{
public:
  Init_MotorRate_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MotorRate_motor_rate header(::rc110_msgs::msg::MotorRate::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_MotorRate_motor_rate(msg_);
  }

private:
  ::rc110_msgs::msg::MotorRate msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::rc110_msgs::msg::MotorRate>()
{
  return rc110_msgs::msg::builder::Init_MotorRate_header();
}

}  // namespace rc110_msgs

#endif  // RC110_MSGS__MSG__DETAIL__MOTOR_RATE__BUILDER_HPP_
