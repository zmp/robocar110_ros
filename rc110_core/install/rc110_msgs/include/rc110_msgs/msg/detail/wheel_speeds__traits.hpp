// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from rc110_msgs:msg/WheelSpeeds.idl
// generated code does not contain a copyright notice

#ifndef RC110_MSGS__MSG__DETAIL__WHEEL_SPEEDS__TRAITS_HPP_
#define RC110_MSGS__MSG__DETAIL__WHEEL_SPEEDS__TRAITS_HPP_

#include "rc110_msgs/msg/detail/wheel_speeds__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<rc110_msgs::msg::WheelSpeeds>()
{
  return "rc110_msgs::msg::WheelSpeeds";
}

template<>
inline const char * name<rc110_msgs::msg::WheelSpeeds>()
{
  return "rc110_msgs/msg/WheelSpeeds";
}

template<>
struct has_fixed_size<rc110_msgs::msg::WheelSpeeds>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<rc110_msgs::msg::WheelSpeeds>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<rc110_msgs::msg::WheelSpeeds>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // RC110_MSGS__MSG__DETAIL__WHEEL_SPEEDS__TRAITS_HPP_
