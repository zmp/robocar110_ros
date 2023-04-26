// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rc110_msgs:msg/Offsets.idl
// generated code does not contain a copyright notice

#ifndef RC110_MSGS__MSG__DETAIL__OFFSETS__STRUCT_H_
#define RC110_MSGS__MSG__DETAIL__OFFSETS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Struct defined in msg/Offsets in the package rc110_msgs.
typedef struct rc110_msgs__msg__Offsets
{
  float gyro;
  float accel_x;
  float accel_y;
  float accel_z;
  float motor_current;
  float steering;
} rc110_msgs__msg__Offsets;

// Struct for a sequence of rc110_msgs__msg__Offsets.
typedef struct rc110_msgs__msg__Offsets__Sequence
{
  rc110_msgs__msg__Offsets * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rc110_msgs__msg__Offsets__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // RC110_MSGS__MSG__DETAIL__OFFSETS__STRUCT_H_
