// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rc110_msgs:msg/Status.idl
// generated code does not contain a copyright notice

#ifndef RC110_MSGS__MSG__DETAIL__STATUS__STRUCT_H_
#define RC110_MSGS__MSG__DETAIL__STATUS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'MOTOR_OFF'.
enum
{
  rc110_msgs__msg__Status__MOTOR_OFF = 0
};

/// Constant 'MOTOR_ON'.
enum
{
  rc110_msgs__msg__Status__MOTOR_ON = 1
};

/// Constant 'MOTOR_NEUTRAL'.
enum
{
  rc110_msgs__msg__Status__MOTOR_NEUTRAL = 2
};

// Struct defined in msg/Status in the package rc110_msgs.
typedef struct rc110_msgs__msg__Status
{
  bool board_enabled;
  uint8_t motor_state;
  uint8_t servo_state;
} rc110_msgs__msg__Status;

// Struct for a sequence of rc110_msgs__msg__Status.
typedef struct rc110_msgs__msg__Status__Sequence
{
  rc110_msgs__msg__Status * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rc110_msgs__msg__Status__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // RC110_MSGS__MSG__DETAIL__STATUS__STRUCT_H_
