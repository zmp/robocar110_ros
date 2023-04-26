// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from rc110_msgs:msg/Offsets.idl
// generated code does not contain a copyright notice
#include "rc110_msgs/msg/detail/offsets__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
rc110_msgs__msg__Offsets__init(rc110_msgs__msg__Offsets * msg)
{
  if (!msg) {
    return false;
  }
  // gyro
  // accel_x
  // accel_y
  // accel_z
  // motor_current
  // steering
  return true;
}

void
rc110_msgs__msg__Offsets__fini(rc110_msgs__msg__Offsets * msg)
{
  if (!msg) {
    return;
  }
  // gyro
  // accel_x
  // accel_y
  // accel_z
  // motor_current
  // steering
}

bool
rc110_msgs__msg__Offsets__are_equal(const rc110_msgs__msg__Offsets * lhs, const rc110_msgs__msg__Offsets * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // gyro
  if (lhs->gyro != rhs->gyro) {
    return false;
  }
  // accel_x
  if (lhs->accel_x != rhs->accel_x) {
    return false;
  }
  // accel_y
  if (lhs->accel_y != rhs->accel_y) {
    return false;
  }
  // accel_z
  if (lhs->accel_z != rhs->accel_z) {
    return false;
  }
  // motor_current
  if (lhs->motor_current != rhs->motor_current) {
    return false;
  }
  // steering
  if (lhs->steering != rhs->steering) {
    return false;
  }
  return true;
}

bool
rc110_msgs__msg__Offsets__copy(
  const rc110_msgs__msg__Offsets * input,
  rc110_msgs__msg__Offsets * output)
{
  if (!input || !output) {
    return false;
  }
  // gyro
  output->gyro = input->gyro;
  // accel_x
  output->accel_x = input->accel_x;
  // accel_y
  output->accel_y = input->accel_y;
  // accel_z
  output->accel_z = input->accel_z;
  // motor_current
  output->motor_current = input->motor_current;
  // steering
  output->steering = input->steering;
  return true;
}

rc110_msgs__msg__Offsets *
rc110_msgs__msg__Offsets__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rc110_msgs__msg__Offsets * msg = (rc110_msgs__msg__Offsets *)allocator.allocate(sizeof(rc110_msgs__msg__Offsets), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(rc110_msgs__msg__Offsets));
  bool success = rc110_msgs__msg__Offsets__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
rc110_msgs__msg__Offsets__destroy(rc110_msgs__msg__Offsets * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    rc110_msgs__msg__Offsets__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
rc110_msgs__msg__Offsets__Sequence__init(rc110_msgs__msg__Offsets__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rc110_msgs__msg__Offsets * data = NULL;

  if (size) {
    data = (rc110_msgs__msg__Offsets *)allocator.zero_allocate(size, sizeof(rc110_msgs__msg__Offsets), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = rc110_msgs__msg__Offsets__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        rc110_msgs__msg__Offsets__fini(&data[i - 1]);
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
rc110_msgs__msg__Offsets__Sequence__fini(rc110_msgs__msg__Offsets__Sequence * array)
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
      rc110_msgs__msg__Offsets__fini(&array->data[i]);
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

rc110_msgs__msg__Offsets__Sequence *
rc110_msgs__msg__Offsets__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rc110_msgs__msg__Offsets__Sequence * array = (rc110_msgs__msg__Offsets__Sequence *)allocator.allocate(sizeof(rc110_msgs__msg__Offsets__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = rc110_msgs__msg__Offsets__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
rc110_msgs__msg__Offsets__Sequence__destroy(rc110_msgs__msg__Offsets__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    rc110_msgs__msg__Offsets__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
rc110_msgs__msg__Offsets__Sequence__are_equal(const rc110_msgs__msg__Offsets__Sequence * lhs, const rc110_msgs__msg__Offsets__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!rc110_msgs__msg__Offsets__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
rc110_msgs__msg__Offsets__Sequence__copy(
  const rc110_msgs__msg__Offsets__Sequence * input,
  rc110_msgs__msg__Offsets__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(rc110_msgs__msg__Offsets);
    rc110_msgs__msg__Offsets * data =
      (rc110_msgs__msg__Offsets *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!rc110_msgs__msg__Offsets__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          rc110_msgs__msg__Offsets__fini(&data[i]);
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
    if (!rc110_msgs__msg__Offsets__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
