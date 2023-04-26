// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from rc110_msgs:msg/WheelSpeeds.idl
// generated code does not contain a copyright notice
#include "rc110_msgs/msg/detail/wheel_speeds__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"

bool
rc110_msgs__msg__WheelSpeeds__init(rc110_msgs__msg__WheelSpeeds * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    rc110_msgs__msg__WheelSpeeds__fini(msg);
    return false;
  }
  // speed_fl
  // speed_fr
  // speed_rl
  // speed_rr
  return true;
}

void
rc110_msgs__msg__WheelSpeeds__fini(rc110_msgs__msg__WheelSpeeds * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // speed_fl
  // speed_fr
  // speed_rl
  // speed_rr
}

bool
rc110_msgs__msg__WheelSpeeds__are_equal(const rc110_msgs__msg__WheelSpeeds * lhs, const rc110_msgs__msg__WheelSpeeds * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // speed_fl
  if (lhs->speed_fl != rhs->speed_fl) {
    return false;
  }
  // speed_fr
  if (lhs->speed_fr != rhs->speed_fr) {
    return false;
  }
  // speed_rl
  if (lhs->speed_rl != rhs->speed_rl) {
    return false;
  }
  // speed_rr
  if (lhs->speed_rr != rhs->speed_rr) {
    return false;
  }
  return true;
}

bool
rc110_msgs__msg__WheelSpeeds__copy(
  const rc110_msgs__msg__WheelSpeeds * input,
  rc110_msgs__msg__WheelSpeeds * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // speed_fl
  output->speed_fl = input->speed_fl;
  // speed_fr
  output->speed_fr = input->speed_fr;
  // speed_rl
  output->speed_rl = input->speed_rl;
  // speed_rr
  output->speed_rr = input->speed_rr;
  return true;
}

rc110_msgs__msg__WheelSpeeds *
rc110_msgs__msg__WheelSpeeds__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rc110_msgs__msg__WheelSpeeds * msg = (rc110_msgs__msg__WheelSpeeds *)allocator.allocate(sizeof(rc110_msgs__msg__WheelSpeeds), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(rc110_msgs__msg__WheelSpeeds));
  bool success = rc110_msgs__msg__WheelSpeeds__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
rc110_msgs__msg__WheelSpeeds__destroy(rc110_msgs__msg__WheelSpeeds * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    rc110_msgs__msg__WheelSpeeds__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
rc110_msgs__msg__WheelSpeeds__Sequence__init(rc110_msgs__msg__WheelSpeeds__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rc110_msgs__msg__WheelSpeeds * data = NULL;

  if (size) {
    data = (rc110_msgs__msg__WheelSpeeds *)allocator.zero_allocate(size, sizeof(rc110_msgs__msg__WheelSpeeds), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = rc110_msgs__msg__WheelSpeeds__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        rc110_msgs__msg__WheelSpeeds__fini(&data[i - 1]);
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
rc110_msgs__msg__WheelSpeeds__Sequence__fini(rc110_msgs__msg__WheelSpeeds__Sequence * array)
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
      rc110_msgs__msg__WheelSpeeds__fini(&array->data[i]);
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

rc110_msgs__msg__WheelSpeeds__Sequence *
rc110_msgs__msg__WheelSpeeds__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rc110_msgs__msg__WheelSpeeds__Sequence * array = (rc110_msgs__msg__WheelSpeeds__Sequence *)allocator.allocate(sizeof(rc110_msgs__msg__WheelSpeeds__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = rc110_msgs__msg__WheelSpeeds__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
rc110_msgs__msg__WheelSpeeds__Sequence__destroy(rc110_msgs__msg__WheelSpeeds__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    rc110_msgs__msg__WheelSpeeds__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
rc110_msgs__msg__WheelSpeeds__Sequence__are_equal(const rc110_msgs__msg__WheelSpeeds__Sequence * lhs, const rc110_msgs__msg__WheelSpeeds__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!rc110_msgs__msg__WheelSpeeds__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
rc110_msgs__msg__WheelSpeeds__Sequence__copy(
  const rc110_msgs__msg__WheelSpeeds__Sequence * input,
  rc110_msgs__msg__WheelSpeeds__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(rc110_msgs__msg__WheelSpeeds);
    rc110_msgs__msg__WheelSpeeds * data =
      (rc110_msgs__msg__WheelSpeeds *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!rc110_msgs__msg__WheelSpeeds__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          rc110_msgs__msg__WheelSpeeds__fini(&data[i]);
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
    if (!rc110_msgs__msg__WheelSpeeds__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
