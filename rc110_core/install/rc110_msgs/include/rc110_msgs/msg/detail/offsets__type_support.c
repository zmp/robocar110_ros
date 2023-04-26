// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from rc110_msgs:msg/Offsets.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "rc110_msgs/msg/detail/offsets__rosidl_typesupport_introspection_c.h"
#include "rc110_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "rc110_msgs/msg/detail/offsets__functions.h"
#include "rc110_msgs/msg/detail/offsets__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void Offsets__rosidl_typesupport_introspection_c__Offsets_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  rc110_msgs__msg__Offsets__init(message_memory);
}

void Offsets__rosidl_typesupport_introspection_c__Offsets_fini_function(void * message_memory)
{
  rc110_msgs__msg__Offsets__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember Offsets__rosidl_typesupport_introspection_c__Offsets_message_member_array[6] = {
  {
    "gyro",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rc110_msgs__msg__Offsets, gyro),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "accel_x",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rc110_msgs__msg__Offsets, accel_x),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "accel_y",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rc110_msgs__msg__Offsets, accel_y),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "accel_z",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rc110_msgs__msg__Offsets, accel_z),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "motor_current",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rc110_msgs__msg__Offsets, motor_current),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "steering",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rc110_msgs__msg__Offsets, steering),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers Offsets__rosidl_typesupport_introspection_c__Offsets_message_members = {
  "rc110_msgs__msg",  // message namespace
  "Offsets",  // message name
  6,  // number of fields
  sizeof(rc110_msgs__msg__Offsets),
  Offsets__rosidl_typesupport_introspection_c__Offsets_message_member_array,  // message members
  Offsets__rosidl_typesupport_introspection_c__Offsets_init_function,  // function to initialize message memory (memory has to be allocated)
  Offsets__rosidl_typesupport_introspection_c__Offsets_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t Offsets__rosidl_typesupport_introspection_c__Offsets_message_type_support_handle = {
  0,
  &Offsets__rosidl_typesupport_introspection_c__Offsets_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_rc110_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rc110_msgs, msg, Offsets)() {
  if (!Offsets__rosidl_typesupport_introspection_c__Offsets_message_type_support_handle.typesupport_identifier) {
    Offsets__rosidl_typesupport_introspection_c__Offsets_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &Offsets__rosidl_typesupport_introspection_c__Offsets_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
