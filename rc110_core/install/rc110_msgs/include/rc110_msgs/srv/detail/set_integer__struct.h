// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rc110_msgs:srv/SetInteger.idl
// generated code does not contain a copyright notice

#ifndef RC110_MSGS__SRV__DETAIL__SET_INTEGER__STRUCT_H_
#define RC110_MSGS__SRV__DETAIL__SET_INTEGER__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Struct defined in srv/SetInteger in the package rc110_msgs.
typedef struct rc110_msgs__srv__SetInteger_Request
{
  int32_t data;
} rc110_msgs__srv__SetInteger_Request;

// Struct for a sequence of rc110_msgs__srv__SetInteger_Request.
typedef struct rc110_msgs__srv__SetInteger_Request__Sequence
{
  rc110_msgs__srv__SetInteger_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rc110_msgs__srv__SetInteger_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'message'
#include "rosidl_runtime_c/string.h"

// Struct defined in srv/SetInteger in the package rc110_msgs.
typedef struct rc110_msgs__srv__SetInteger_Response
{
  bool success;
  rosidl_runtime_c__String message;
} rc110_msgs__srv__SetInteger_Response;

// Struct for a sequence of rc110_msgs__srv__SetInteger_Response.
typedef struct rc110_msgs__srv__SetInteger_Response__Sequence
{
  rc110_msgs__srv__SetInteger_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rc110_msgs__srv__SetInteger_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // RC110_MSGS__SRV__DETAIL__SET_INTEGER__STRUCT_H_
