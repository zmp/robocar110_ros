// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from rc110_msgs:msg/Status.idl
// generated code does not contain a copyright notice

#ifndef RC110_MSGS__MSG__DETAIL__STATUS__TRAITS_HPP_
#define RC110_MSGS__MSG__DETAIL__STATUS__TRAITS_HPP_

#include "rc110_msgs/msg/detail/status__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<rc110_msgs::msg::Status>()
{
  return "rc110_msgs::msg::Status";
}

template<>
inline const char * name<rc110_msgs::msg::Status>()
{
  return "rc110_msgs/msg/Status";
}

template<>
struct has_fixed_size<rc110_msgs::msg::Status>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<rc110_msgs::msg::Status>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<rc110_msgs::msg::Status>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // RC110_MSGS__MSG__DETAIL__STATUS__TRAITS_HPP_
