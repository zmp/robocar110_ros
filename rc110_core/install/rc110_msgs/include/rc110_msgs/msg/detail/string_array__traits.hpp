// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from rc110_msgs:msg/StringArray.idl
// generated code does not contain a copyright notice

#ifndef RC110_MSGS__MSG__DETAIL__STRING_ARRAY__TRAITS_HPP_
#define RC110_MSGS__MSG__DETAIL__STRING_ARRAY__TRAITS_HPP_

#include "rc110_msgs/msg/detail/string_array__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<rc110_msgs::msg::StringArray>()
{
  return "rc110_msgs::msg::StringArray";
}

template<>
inline const char * name<rc110_msgs::msg::StringArray>()
{
  return "rc110_msgs/msg/StringArray";
}

template<>
struct has_fixed_size<rc110_msgs::msg::StringArray>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<rc110_msgs::msg::StringArray>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<rc110_msgs::msg::StringArray>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // RC110_MSGS__MSG__DETAIL__STRING_ARRAY__TRAITS_HPP_
