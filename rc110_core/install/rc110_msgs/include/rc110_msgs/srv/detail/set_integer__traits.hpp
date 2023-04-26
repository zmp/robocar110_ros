// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from rc110_msgs:srv/SetInteger.idl
// generated code does not contain a copyright notice

#ifndef RC110_MSGS__SRV__DETAIL__SET_INTEGER__TRAITS_HPP_
#define RC110_MSGS__SRV__DETAIL__SET_INTEGER__TRAITS_HPP_

#include "rc110_msgs/srv/detail/set_integer__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<rc110_msgs::srv::SetInteger_Request>()
{
  return "rc110_msgs::srv::SetInteger_Request";
}

template<>
inline const char * name<rc110_msgs::srv::SetInteger_Request>()
{
  return "rc110_msgs/srv/SetInteger_Request";
}

template<>
struct has_fixed_size<rc110_msgs::srv::SetInteger_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<rc110_msgs::srv::SetInteger_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<rc110_msgs::srv::SetInteger_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<rc110_msgs::srv::SetInteger_Response>()
{
  return "rc110_msgs::srv::SetInteger_Response";
}

template<>
inline const char * name<rc110_msgs::srv::SetInteger_Response>()
{
  return "rc110_msgs/srv/SetInteger_Response";
}

template<>
struct has_fixed_size<rc110_msgs::srv::SetInteger_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<rc110_msgs::srv::SetInteger_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<rc110_msgs::srv::SetInteger_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<rc110_msgs::srv::SetInteger>()
{
  return "rc110_msgs::srv::SetInteger";
}

template<>
inline const char * name<rc110_msgs::srv::SetInteger>()
{
  return "rc110_msgs/srv/SetInteger";
}

template<>
struct has_fixed_size<rc110_msgs::srv::SetInteger>
  : std::integral_constant<
    bool,
    has_fixed_size<rc110_msgs::srv::SetInteger_Request>::value &&
    has_fixed_size<rc110_msgs::srv::SetInteger_Response>::value
  >
{
};

template<>
struct has_bounded_size<rc110_msgs::srv::SetInteger>
  : std::integral_constant<
    bool,
    has_bounded_size<rc110_msgs::srv::SetInteger_Request>::value &&
    has_bounded_size<rc110_msgs::srv::SetInteger_Response>::value
  >
{
};

template<>
struct is_service<rc110_msgs::srv::SetInteger>
  : std::true_type
{
};

template<>
struct is_service_request<rc110_msgs::srv::SetInteger_Request>
  : std::true_type
{
};

template<>
struct is_service_response<rc110_msgs::srv::SetInteger_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // RC110_MSGS__SRV__DETAIL__SET_INTEGER__TRAITS_HPP_
