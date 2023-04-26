// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rc110_msgs:msg/WheelSpeeds.idl
// generated code does not contain a copyright notice

#ifndef RC110_MSGS__MSG__DETAIL__WHEEL_SPEEDS__STRUCT_H_
#define RC110_MSGS__MSG__DETAIL__WHEEL_SPEEDS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"

// Struct defined in msg/WheelSpeeds in the package rc110_msgs.
typedef struct rc110_msgs__msg__WheelSpeeds
{
  std_msgs__msg__Header header;
  float speed_fl;
  float speed_fr;
  float speed_rl;
  float speed_rr;
} rc110_msgs__msg__WheelSpeeds;

// Struct for a sequence of rc110_msgs__msg__WheelSpeeds.
typedef struct rc110_msgs__msg__WheelSpeeds__Sequence
{
  rc110_msgs__msg__WheelSpeeds * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rc110_msgs__msg__WheelSpeeds__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // RC110_MSGS__MSG__DETAIL__WHEEL_SPEEDS__STRUCT_H_
