// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rc110_msgs:msg/Status.idl
// generated code does not contain a copyright notice

#ifndef RC110_MSGS__MSG__DETAIL__STATUS__BUILDER_HPP_
#define RC110_MSGS__MSG__DETAIL__STATUS__BUILDER_HPP_

#include "rc110_msgs/msg/detail/status__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace rc110_msgs
{

namespace msg
{

namespace builder
{

class Init_Status_servo_state
{
public:
  explicit Init_Status_servo_state(::rc110_msgs::msg::Status & msg)
  : msg_(msg)
  {}
  ::rc110_msgs::msg::Status servo_state(::rc110_msgs::msg::Status::_servo_state_type arg)
  {
    msg_.servo_state = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rc110_msgs::msg::Status msg_;
};

class Init_Status_motor_state
{
public:
  explicit Init_Status_motor_state(::rc110_msgs::msg::Status & msg)
  : msg_(msg)
  {}
  Init_Status_servo_state motor_state(::rc110_msgs::msg::Status::_motor_state_type arg)
  {
    msg_.motor_state = std::move(arg);
    return Init_Status_servo_state(msg_);
  }

private:
  ::rc110_msgs::msg::Status msg_;
};

class Init_Status_board_enabled
{
public:
  Init_Status_board_enabled()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Status_motor_state board_enabled(::rc110_msgs::msg::Status::_board_enabled_type arg)
  {
    msg_.board_enabled = std::move(arg);
    return Init_Status_motor_state(msg_);
  }

private:
  ::rc110_msgs::msg::Status msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::rc110_msgs::msg::Status>()
{
  return rc110_msgs::msg::builder::Init_Status_board_enabled();
}

}  // namespace rc110_msgs

#endif  // RC110_MSGS__MSG__DETAIL__STATUS__BUILDER_HPP_
