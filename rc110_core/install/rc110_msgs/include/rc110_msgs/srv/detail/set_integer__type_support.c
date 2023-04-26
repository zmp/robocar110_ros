// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from rc110_msgs:srv/SetInteger.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "rc110_msgs/srv/detail/set_integer__rosidl_typesupport_introspection_c.h"
#include "rc110_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "rc110_msgs/srv/detail/set_integer__functions.h"
#include "rc110_msgs/srv/detail/set_integer__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void SetInteger_Request__rosidl_typesupport_introspection_c__SetInteger_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  rc110_msgs__srv__SetInteger_Request__init(message_memory);
}

void SetInteger_Request__rosidl_typesupport_introspection_c__SetInteger_Request_fini_function(void * message_memory)
{
  rc110_msgs__srv__SetInteger_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember SetInteger_Request__rosidl_typesupport_introspection_c__SetInteger_Request_message_member_array[1] = {
  {
    "data",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rc110_msgs__srv__SetInteger_Request, data),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers SetInteger_Request__rosidl_typesupport_introspection_c__SetInteger_Request_message_members = {
  "rc110_msgs__srv",  // message namespace
  "SetInteger_Request",  // message name
  1,  // number of fields
  sizeof(rc110_msgs__srv__SetInteger_Request),
  SetInteger_Request__rosidl_typesupport_introspection_c__SetInteger_Request_message_member_array,  // message members
  SetInteger_Request__rosidl_typesupport_introspection_c__SetInteger_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  SetInteger_Request__rosidl_typesupport_introspection_c__SetInteger_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t SetInteger_Request__rosidl_typesupport_introspection_c__SetInteger_Request_message_type_support_handle = {
  0,
  &SetInteger_Request__rosidl_typesupport_introspection_c__SetInteger_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_rc110_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rc110_msgs, srv, SetInteger_Request)() {
  if (!SetInteger_Request__rosidl_typesupport_introspection_c__SetInteger_Request_message_type_support_handle.typesupport_identifier) {
    SetInteger_Request__rosidl_typesupport_introspection_c__SetInteger_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &SetInteger_Request__rosidl_typesupport_introspection_c__SetInteger_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "rc110_msgs/srv/detail/set_integer__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rc110_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "rc110_msgs/srv/detail/set_integer__functions.h"
// already included above
// #include "rc110_msgs/srv/detail/set_integer__struct.h"


// Include directives for member types
// Member `message`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void SetInteger_Response__rosidl_typesupport_introspection_c__SetInteger_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  rc110_msgs__srv__SetInteger_Response__init(message_memory);
}

void SetInteger_Response__rosidl_typesupport_introspection_c__SetInteger_Response_fini_function(void * message_memory)
{
  rc110_msgs__srv__SetInteger_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember SetInteger_Response__rosidl_typesupport_introspection_c__SetInteger_Response_message_member_array[2] = {
  {
    "success",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rc110_msgs__srv__SetInteger_Response, success),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "message",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rc110_msgs__srv__SetInteger_Response, message),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers SetInteger_Response__rosidl_typesupport_introspection_c__SetInteger_Response_message_members = {
  "rc110_msgs__srv",  // message namespace
  "SetInteger_Response",  // message name
  2,  // number of fields
  sizeof(rc110_msgs__srv__SetInteger_Response),
  SetInteger_Response__rosidl_typesupport_introspection_c__SetInteger_Response_message_member_array,  // message members
  SetInteger_Response__rosidl_typesupport_introspection_c__SetInteger_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  SetInteger_Response__rosidl_typesupport_introspection_c__SetInteger_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t SetInteger_Response__rosidl_typesupport_introspection_c__SetInteger_Response_message_type_support_handle = {
  0,
  &SetInteger_Response__rosidl_typesupport_introspection_c__SetInteger_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_rc110_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rc110_msgs, srv, SetInteger_Response)() {
  if (!SetInteger_Response__rosidl_typesupport_introspection_c__SetInteger_Response_message_type_support_handle.typesupport_identifier) {
    SetInteger_Response__rosidl_typesupport_introspection_c__SetInteger_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &SetInteger_Response__rosidl_typesupport_introspection_c__SetInteger_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "rc110_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rc110_msgs/srv/detail/set_integer__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers rc110_msgs__srv__detail__set_integer__rosidl_typesupport_introspection_c__SetInteger_service_members = {
  "rc110_msgs__srv",  // service namespace
  "SetInteger",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // rc110_msgs__srv__detail__set_integer__rosidl_typesupport_introspection_c__SetInteger_Request_message_type_support_handle,
  NULL  // response message
  // rc110_msgs__srv__detail__set_integer__rosidl_typesupport_introspection_c__SetInteger_Response_message_type_support_handle
};

static rosidl_service_type_support_t rc110_msgs__srv__detail__set_integer__rosidl_typesupport_introspection_c__SetInteger_service_type_support_handle = {
  0,
  &rc110_msgs__srv__detail__set_integer__rosidl_typesupport_introspection_c__SetInteger_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rc110_msgs, srv, SetInteger_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rc110_msgs, srv, SetInteger_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_rc110_msgs
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rc110_msgs, srv, SetInteger)() {
  if (!rc110_msgs__srv__detail__set_integer__rosidl_typesupport_introspection_c__SetInteger_service_type_support_handle.typesupport_identifier) {
    rc110_msgs__srv__detail__set_integer__rosidl_typesupport_introspection_c__SetInteger_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)rc110_msgs__srv__detail__set_integer__rosidl_typesupport_introspection_c__SetInteger_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rc110_msgs, srv, SetInteger_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rc110_msgs, srv, SetInteger_Response)()->data;
  }

  return &rc110_msgs__srv__detail__set_integer__rosidl_typesupport_introspection_c__SetInteger_service_type_support_handle;
}
