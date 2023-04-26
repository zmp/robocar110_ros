// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from rc110_msgs:msg/MotorRate.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "rc110_msgs/msg/detail/motor_rate__rosidl_typesupport_introspection_c.h"
#include "rc110_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "rc110_msgs/msg/detail/motor_rate__functions.h"
#include "rc110_msgs/msg/detail/motor_rate__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void MotorRate__rosidl_typesupport_introspection_c__MotorRate_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  rc110_msgs__msg__MotorRate__init(message_memory);
}

void MotorRate__rosidl_typesupport_introspection_c__MotorRate_fini_function(void * message_memory)
{
  rc110_msgs__msg__MotorRate__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember MotorRate__rosidl_typesupport_introspection_c__MotorRate_message_member_array[3] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rc110_msgs__msg__MotorRate, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "motor_rate",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rc110_msgs__msg__MotorRate, motor_rate),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "estimated_speed",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rc110_msgs__msg__MotorRate, estimated_speed),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers MotorRate__rosidl_typesupport_introspection_c__MotorRate_message_members = {
  "rc110_msgs__msg",  // message namespace
  "MotorRate",  // message name
  3,  // number of fields
  sizeof(rc110_msgs__msg__MotorRate),
  MotorRate__rosidl_typesupport_introspection_c__MotorRate_message_member_array,  // message members
  MotorRate__rosidl_typesupport_introspection_c__MotorRate_init_function,  // function to initialize message memory (memory has to be allocated)
  MotorRate__rosidl_typesupport_introspection_c__MotorRate_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t MotorRate__rosidl_typesupport_introspection_c__MotorRate_message_type_support_handle = {
  0,
  &MotorRate__rosidl_typesupport_introspection_c__MotorRate_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_rc110_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rc110_msgs, msg, MotorRate)() {
  MotorRate__rosidl_typesupport_introspection_c__MotorRate_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  if (!MotorRate__rosidl_typesupport_introspection_c__MotorRate_message_type_support_handle.typesupport_identifier) {
    MotorRate__rosidl_typesupport_introspection_c__MotorRate_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &MotorRate__rosidl_typesupport_introspection_c__MotorRate_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
