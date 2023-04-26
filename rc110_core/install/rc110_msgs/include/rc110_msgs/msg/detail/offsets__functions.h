// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from rc110_msgs:msg/Offsets.idl
// generated code does not contain a copyright notice

#ifndef RC110_MSGS__MSG__DETAIL__OFFSETS__FUNCTIONS_H_
#define RC110_MSGS__MSG__DETAIL__OFFSETS__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "rc110_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "rc110_msgs/msg/detail/offsets__struct.h"

/// Initialize msg/Offsets message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * rc110_msgs__msg__Offsets
 * )) before or use
 * rc110_msgs__msg__Offsets__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_rc110_msgs
bool
rc110_msgs__msg__Offsets__init(rc110_msgs__msg__Offsets * msg);

/// Finalize msg/Offsets message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rc110_msgs
void
rc110_msgs__msg__Offsets__fini(rc110_msgs__msg__Offsets * msg);

/// Create msg/Offsets message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * rc110_msgs__msg__Offsets__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_rc110_msgs
rc110_msgs__msg__Offsets *
rc110_msgs__msg__Offsets__create();

/// Destroy msg/Offsets message.
/**
 * It calls
 * rc110_msgs__msg__Offsets__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rc110_msgs
void
rc110_msgs__msg__Offsets__destroy(rc110_msgs__msg__Offsets * msg);

/// Check for msg/Offsets message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_rc110_msgs
bool
rc110_msgs__msg__Offsets__are_equal(const rc110_msgs__msg__Offsets * lhs, const rc110_msgs__msg__Offsets * rhs);

/// Copy a msg/Offsets message.
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
rc110_msgs__msg__Offsets__copy(
  const rc110_msgs__msg__Offsets * input,
  rc110_msgs__msg__Offsets * output);

/// Initialize array of msg/Offsets messages.
/**
 * It allocates the memory for the number of elements and calls
 * rc110_msgs__msg__Offsets__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_rc110_msgs
bool
rc110_msgs__msg__Offsets__Sequence__init(rc110_msgs__msg__Offsets__Sequence * array, size_t size);

/// Finalize array of msg/Offsets messages.
/**
 * It calls
 * rc110_msgs__msg__Offsets__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rc110_msgs
void
rc110_msgs__msg__Offsets__Sequence__fini(rc110_msgs__msg__Offsets__Sequence * array);

/// Create array of msg/Offsets messages.
/**
 * It allocates the memory for the array and calls
 * rc110_msgs__msg__Offsets__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_rc110_msgs
rc110_msgs__msg__Offsets__Sequence *
rc110_msgs__msg__Offsets__Sequence__create(size_t size);

/// Destroy array of msg/Offsets messages.
/**
 * It calls
 * rc110_msgs__msg__Offsets__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rc110_msgs
void
rc110_msgs__msg__Offsets__Sequence__destroy(rc110_msgs__msg__Offsets__Sequence * array);

/// Check for msg/Offsets message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_rc110_msgs
bool
rc110_msgs__msg__Offsets__Sequence__are_equal(const rc110_msgs__msg__Offsets__Sequence * lhs, const rc110_msgs__msg__Offsets__Sequence * rhs);

/// Copy an array of msg/Offsets messages.
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
rc110_msgs__msg__Offsets__Sequence__copy(
  const rc110_msgs__msg__Offsets__Sequence * input,
  rc110_msgs__msg__Offsets__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // RC110_MSGS__MSG__DETAIL__OFFSETS__FUNCTIONS_H_
