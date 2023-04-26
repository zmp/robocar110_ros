// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from rc110_msgs:msg/BaseboardError.idl
// generated code does not contain a copyright notice
#include "rc110_msgs/msg/detail/baseboard_error__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
rc110_msgs__msg__BaseboardError__init(rc110_msgs__msg__BaseboardError * msg)
{
  if (!msg) {
    return false;
  }
  // data
  return true;
}

void
rc110_msgs__msg__BaseboardError__fini(rc110_msgs__msg__BaseboardError * msg)
{
  if (!msg) {
    return;
  }
  // data
}

bool
rc110_msgs__msg__BaseboardError__are_equal(const rc110_msgs__msg__BaseboardError * lhs, const rc110_msgs__msg__BaseboardError * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // data
  if (lhs->data != rhs->data) {
    return false;
  }
  return true;
}

bool
rc110_msgs__msg__BaseboardError__copy(
  const rc110_msgs__msg__BaseboardError * input,
  rc110_msgs__msg__BaseboardError * output)
{
  if (!input || !output) {
    return false;
  }
  // data
  output->data = input->data;
  return true;
}

rc110_msgs__msg__BaseboardError *
rc110_msgs__msg__BaseboardError__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rc110_msgs__msg__BaseboardError * msg = (rc110_msgs__msg__BaseboardError *)allocator.allocate(sizeof(rc110_msgs__msg__BaseboardError), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(rc110_msgs__msg__BaseboardError));
  bool success = rc110_msgs__msg__BaseboardError__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
rc110_msgs__msg__BaseboardError__destroy(rc110_msgs__msg__BaseboardError * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    rc110_msgs__msg__BaseboardError__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
rc110_msgs__msg__BaseboardError__Sequence__init(rc110_msgs__msg__BaseboardError__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rc110_msgs__msg__BaseboardError * data = NULL;

  if (size) {
    data = (rc110_msgs__msg__BaseboardError *)allocator.zero_allocate(size, sizeof(rc110_msgs__msg__BaseboardError), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = rc110_msgs__msg__BaseboardError__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        rc110_msgs__msg__BaseboardError__fini(&data[i - 1]);
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
rc110_msgs__msg__BaseboardError__Sequence__fini(rc110_msgs__msg__BaseboardError__Sequence * array)
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
      rc110_msgs__msg__BaseboardError__fini(&array->data[i]);
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

rc110_msgs__msg__BaseboardError__Sequence *
rc110_msgs__msg__BaseboardError__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rc110_msgs__msg__BaseboardError__Sequence * array = (rc110_msgs__msg__BaseboardError__Sequence *)allocator.allocate(sizeof(rc110_msgs__msg__BaseboardError__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = rc110_msgs__msg__BaseboardError__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
rc110_msgs__msg__BaseboardError__Sequence__destroy(rc110_msgs__msg__BaseboardError__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    rc110_msgs__msg__BaseboardError__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
rc110_msgs__msg__BaseboardError__Sequence__are_equal(const rc110_msgs__msg__BaseboardError__Sequence * lhs, const rc110_msgs__msg__BaseboardError__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!rc110_msgs__msg__BaseboardError__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
rc110_msgs__msg__BaseboardError__Sequence__copy(
  const rc110_msgs__msg__BaseboardError__Sequence * input,
  rc110_msgs__msg__BaseboardError__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(rc110_msgs__msg__BaseboardError);
    rc110_msgs__msg__BaseboardError * data =
      (rc110_msgs__msg__BaseboardError *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!rc110_msgs__msg__BaseboardError__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          rc110_msgs__msg__BaseboardError__fini(&data[i]);
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
    if (!rc110_msgs__msg__BaseboardError__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
