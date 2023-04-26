// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from rc110_msgs:msg/Status.idl
// generated code does not contain a copyright notice
#include "rc110_msgs/msg/detail/status__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
rc110_msgs__msg__Status__init(rc110_msgs__msg__Status * msg)
{
  if (!msg) {
    return false;
  }
  // board_enabled
  // motor_state
  // servo_state
  return true;
}

void
rc110_msgs__msg__Status__fini(rc110_msgs__msg__Status * msg)
{
  if (!msg) {
    return;
  }
  // board_enabled
  // motor_state
  // servo_state
}

bool
rc110_msgs__msg__Status__are_equal(const rc110_msgs__msg__Status * lhs, const rc110_msgs__msg__Status * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // board_enabled
  if (lhs->board_enabled != rhs->board_enabled) {
    return false;
  }
  // motor_state
  if (lhs->motor_state != rhs->motor_state) {
    return false;
  }
  // servo_state
  if (lhs->servo_state != rhs->servo_state) {
    return false;
  }
  return true;
}

bool
rc110_msgs__msg__Status__copy(
  const rc110_msgs__msg__Status * input,
  rc110_msgs__msg__Status * output)
{
  if (!input || !output) {
    return false;
  }
  // board_enabled
  output->board_enabled = input->board_enabled;
  // motor_state
  output->motor_state = input->motor_state;
  // servo_state
  output->servo_state = input->servo_state;
  return true;
}

rc110_msgs__msg__Status *
rc110_msgs__msg__Status__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rc110_msgs__msg__Status * msg = (rc110_msgs__msg__Status *)allocator.allocate(sizeof(rc110_msgs__msg__Status), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(rc110_msgs__msg__Status));
  bool success = rc110_msgs__msg__Status__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
rc110_msgs__msg__Status__destroy(rc110_msgs__msg__Status * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    rc110_msgs__msg__Status__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
rc110_msgs__msg__Status__Sequence__init(rc110_msgs__msg__Status__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rc110_msgs__msg__Status * data = NULL;

  if (size) {
    data = (rc110_msgs__msg__Status *)allocator.zero_allocate(size, sizeof(rc110_msgs__msg__Status), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = rc110_msgs__msg__Status__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        rc110_msgs__msg__Status__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
rc110_msgs__msg__Status__Sequence__fini(rc110_msgs__msg__Status__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      rc110_msgs__msg__Status__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

rc110_msgs__msg__Status__Sequence *
rc110_msgs__msg__Status__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rc110_msgs__msg__Status__Sequence * array = (rc110_msgs__msg__Status__Sequence *)allocator.allocate(sizeof(rc110_msgs__msg__Status__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = rc110_msgs__msg__Status__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
rc110_msgs__msg__Status__Sequence__destroy(rc110_msgs__msg__Status__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    rc110_msgs__msg__Status__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
rc110_msgs__msg__Status__Sequence__are_equal(const rc110_msgs__msg__Status__Sequence * lhs, const rc110_msgs__msg__Status__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!rc110_msgs__msg__Status__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
rc110_msgs__msg__Status__Sequence__copy(
  const rc110_msgs__msg__Status__Sequence * input,
  rc110_msgs__msg__Status__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(rc110_msgs__msg__Status);
    rc110_msgs__msg__Status * data =
      (rc110_msgs__msg__Status *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!rc110_msgs__msg__Status__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          rc110_msgs__msg__Status__fini(&data[i]);
        }
        free(data);
        return false;
      }
    }
    output->data = data;
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!rc110_msgs__msg__Status__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
