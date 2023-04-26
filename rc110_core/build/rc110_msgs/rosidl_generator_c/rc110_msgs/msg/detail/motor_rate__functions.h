// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from rc110_msgs:msg/MotorRate.idl
// generated code does not contain a copyright notice

#ifndef RC110_MSGS__MSG__DETAIL__MOTOR_RATE__FUNCTIONS_H_
#define RC110_MSGS__MSG__DETAIL__MOTOR_RATE__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "rc110_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "rc110_msgs/msg/detail/motor_rate__struct.h"

/// Initialize msg/MotorRate message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * rc110_msgs__msg__MotorRate
 * )) before or use
 * rc110_msgs__msg__MotorRate__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_rc110_msgs
bool
rc110_msgs__msg__MotorRate__init(rc110_msgs__msg__MotorRate * msg);

/// Finalize msg/MotorRate message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rc110_msgs
void
rc110_msgs__msg__MotorRate__fini(rc110_msgs__msg__MotorRate * msg);

/// Create msg/MotorRate message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * rc110_msgs__msg__MotorRate__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_rc110_msgs
rc110_msgs__msg__MotorRate *
rc110_msgs__msg__MotorRate__create();

/// Destroy msg/MotorRate message.
/**
 * It calls
 * rc110_msgs__msg__MotorRate__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rc110_msgs
void
rc110_msgs__msg__MotorRate__destroy(rc110_msgs__msg__MotorRate * msg);

/// Check for msg/MotorRate message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_rc110_msgs
bool
rc110_msgs__msg__MotorRate__are_equal(const rc110_msgs__msg__MotorRate * lhs, const rc110_msgs__msg__MotorRate * rhs);

/// Copy a msg/MotorRate message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_rc110_msgs
bool
rc110_msgs__msg__MotorRate__copy(
  const rc110_msgs__msg__MotorRate * input,
  rc110_msgs__msg__MotorRate * output);

/// Initialize array of msg/MotorRate messages.
/**
 * It allocates the memory for the number of elements and calls
 * rc110_msgs__msg__MotorRate__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_rc110_msgs
bool
rc110_msgs__msg__MotorRate__Sequence__init(rc110_msgs__msg__MotorRate__Sequence * array, size_t size);

/// Finalize array of msg/MotorRate messages.
/**
 * It calls
 * rc110_msgs__msg__MotorRate__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rc110_msgs
void
rc110_msgs__msg__MotorRate__Sequence__fini(rc110_msgs__msg__MotorRate__Sequence * array);

/// Create array of msg/MotorRate messages.
/**
 * It allocates the memory for the array and calls
 * rc110_msgs__msg__MotorRate__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_rc110_msgs
rc110_msgs__msg__MotorRate__Sequence *
rc110_msgs__msg__MotorRate__Sequence__create(size_t size);

/// Destroy array of msg/MotorRate messages.
/**
 * It calls
 * rc110_msgs__msg__MotorRate__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rc110_msgs
void
rc110_msgs__msg__MotorRate__Sequence__destroy(rc110_msgs__msg__MotorRate__Sequence * array);

/// Check for msg/MotorRate message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_rc110_msgs
bool
rc110_msgs__msg__MotorRate__Sequence__are_equal(const rc110_msgs__msg__MotorRate__Sequence * lhs, const rc110_msgs__msg__MotorRate__Sequence * rhs);

/// Copy an array of msg/MotorRate messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_rc110_msgs
bool
rc110_msgs__msg__MotorRate__Sequence__copy(
  const rc110_msgs__msg__MotorRate__Sequence * input,
  rc110_msgs__msg__MotorRate__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // RC110_MSGS__MSG__DETAIL__MOTOR_RATE__FUNCTIONS_H_
