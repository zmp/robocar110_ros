// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rc110_msgs:msg/Offsets.idl
// generated code does not contain a copyright notice

#ifndef RC110_MSGS__MSG__DETAIL__OFFSETS__BUILDER_HPP_
#define RC110_MSGS__MSG__DETAIL__OFFSETS__BUILDER_HPP_

#include "rc110_msgs/msg/detail/offsets__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace rc110_msgs
{

namespace msg
{

namespace builder
{

class Init_Offsets_steering
{
public:
  explicit Init_Offsets_steering(::rc110_msgs::msg::Offsets & msg)
  : msg_(msg)
  {}
  ::rc110_msgs::msg::Offsets steering(::rc110_msgs::msg::Offsets::_steering_type arg)
  {
    msg_.steering = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rc110_msgs::msg::Offsets msg_;
};

class Init_Offsets_motor_current
{
public:
  explicit Init_Offsets_motor_current(::rc110_msgs::msg::Offsets & msg)
  : msg_(msg)
  {}
  Init_Offsets_steering motor_current(::rc110_msgs::msg::Offsets::_motor_current_type arg)
  {
    msg_.motor_current = std::move(arg);
    return Init_Offsets_steering(msg_);
  }

private:
  ::rc110_msgs::msg::Offsets msg_;
};

class Init_Offsets_accel_z
{
public:
  explicit Init_Offsets_accel_z(::rc110_msgs::msg::Offsets & msg)
  : msg_(msg)
  {}
  Init_Offsets_motor_current accel_z(::rc110_msgs::msg::Offsets::_accel_z_type arg)
  {
    msg_.accel_z = std::move(arg);
    return Init_Offsets_motor_current(msg_);
  }

private:
  ::rc110_msgs::msg::Offsets msg_;
};

class Init_Offsets_accel_y
{
public:
  explicit Init_Offsets_accel_y(::rc110_msgs::msg::Offsets & msg)
  : msg_(msg)
  {}
  Init_Offsets_accel_z accel_y(::rc110_msgs::msg::Offsets::_accel_y_type arg)
  {
    msg_.accel_y = std::move(arg);
    return Init_Offsets_accel_z(msg_);
  }

private:
  ::rc110_msgs::msg::Offsets msg_;
};

class Init_Offsets_accel_x
{
public:
  explicit Init_Offsets_accel_x(::rc110_msgs::msg::Offsets & msg)
  : msg_(msg)
  {}
  Init_Offsets_accel_y accel_x(::rc110_msgs::msg::Offsets::_accel_x_type arg)
  {
    msg_.accel_x = std::move(arg);
    return Init_Offsets_accel_y(msg_);
  }

private:
  ::rc110_msgs::msg::Offsets msg_;
};

class Init_Offsets_gyro
{
public:
  Init_Offsets_gyro()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Offsets_accel_x gyro(::rc110_msgs::msg::Offsets::_gyro_type arg)
  {
    msg_.gyro = std::move(arg);
    return Init_Offsets_accel_x(msg_);
  }

private:
  ::rc110_msgs::msg::Offsets msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::rc110_msgs::msg::Offsets>()
{
  return rc110_msgs::msg::builder::Init_Offsets_gyro();
}

}  // namespace rc110_msgs

#endif  // RC110_MSGS__MSG__DETAIL__OFFSETS__BUILDER_HPP_
