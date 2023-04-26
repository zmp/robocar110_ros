// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rc110_msgs:msg/BaseboardError.idl
// generated code does not contain a copyright notice

#ifndef RC110_MSGS__MSG__DETAIL__BASEBOARD_ERROR__STRUCT_H_
#define RC110_MSGS__MSG__DETAIL__BASEBOARD_ERROR__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'NONE'.
enum
{
  rc110_msgs__msg__BaseboardError__NONE = 0
};

/// Constant 'BOARD_HEAT'.
enum
{
  rc110_msgs__msg__BaseboardError__BOARD_HEAT = 1
};

/// Constant 'MOTOR_HEAT'.
enum
{
  rc110_msgs__msg__BaseboardError__MOTOR_HEAT = 2
};

/// Constant 'MOTOR_FAILURE'.
enum
{
  rc110_msgs__msg__BaseboardError__MOTOR_FAILURE = 3
};

/// Constant 'LOW_VOLTAGE'.
enum
{
  rc110_msgs__msg__BaseboardError__LOW_VOLTAGE = 4
};

// Struct defined in msg/BaseboardError in the package rc110_msgs.
typedef struct rc110_msgs__msg__BaseboardError
{
  uint8_t data;
} rc110_msgs__msg__BaseboardError;

// Struct for a sequence of rc110_msgs__msg__BaseboardError.
typedef struct rc110_msgs__msg__BaseboardError__Sequence
{
  rc110_msgs__msg__BaseboardError * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rc110_msgs__msg__BaseboardError__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // RC110_MSGS__MSG__DETAIL__BASEBOARD_ERROR__STRUCT_H_
