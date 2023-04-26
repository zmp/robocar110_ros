// generated from rosidl_typesupport_cpp/resource/idl__type_support.cpp.em
// with input from rc110_msgs:srv/SetInteger.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rc110_msgs/srv/detail/set_integer__struct.hpp"
#include "rosidl_typesupport_cpp/identifier.hpp"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
#include "rosidl_typesupport_cpp/visibility_control.h"
#include "rosidl_typesupport_interface/macros.h"

namespace rc110_msgs
{

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _SetInteger_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _SetInteger_Request_type_support_ids_t;

static const _SetInteger_Request_type_support_ids_t _SetInteger_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _SetInteger_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _SetInteger_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _SetInteger_Request_type_support_symbol_names_t _SetInteger_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, rc110_msgs, srv, SetInteger_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, rc110_msgs, srv, SetInteger_Request)),
  }
};

typedef struct _SetInteger_Request_type_support_data_t
{
  void * data[2];
} _SetInteger_Request_type_support_data_t;

static _SetInteger_Request_type_support_data_t _SetInteger_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _SetInteger_Request_message_typesupport_map = {
  2,
  "rc110_msgs",
  &_SetInteger_Request_message_typesupport_ids.typesupport_identifier[0],
  &_SetInteger_Request_message_typesupport_symbol_names.symbol_name[0],
  &_SetInteger_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t SetInteger_Request_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_SetInteger_Request_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace rc110_msgs

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<rc110_msgs::srv::SetInteger_Request>()
{
  return &::rc110_msgs::srv::rosidl_typesupport_cpp::SetInteger_Request_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, rc110_msgs, srv, SetInteger_Request)() {
  return get_message_type_support_handle<rc110_msgs::srv::SetInteger_Request>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "rc110_msgs/srv/detail/set_integer__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace rc110_msgs
{

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _SetInteger_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _SetInteger_Response_type_support_ids_t;

static const _SetInteger_Response_type_support_ids_t _SetInteger_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _SetInteger_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _SetInteger_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _SetInteger_Response_type_support_symbol_names_t _SetInteger_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, rc110_msgs, srv, SetInteger_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, rc110_msgs, srv, SetInteger_Response)),
  }
};

typedef struct _SetInteger_Response_type_support_data_t
{
  void * data[2];
} _SetInteger_Response_type_support_data_t;

static _SetInteger_Response_type_support_data_t _SetInteger_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _SetInteger_Response_message_typesupport_map = {
  2,
  "rc110_msgs",
  &_SetInteger_Response_message_typesupport_ids.typesupport_identifier[0],
  &_SetInteger_Response_message_typesupport_symbol_names.symbol_name[0],
  &_SetInteger_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t SetInteger_Response_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_SetInteger_Response_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace rc110_msgs

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<rc110_msgs::srv::SetInteger_Response>()
{
  return &::rc110_msgs::srv::rosidl_typesupport_cpp::SetInteger_Response_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, rc110_msgs, srv, SetInteger_Response)() {
  return get_message_type_support_handle<rc110_msgs::srv::SetInteger_Response>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "rc110_msgs/srv/detail/set_integer__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_cpp/service_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace rc110_msgs
{

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _SetInteger_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _SetInteger_type_support_ids_t;

static const _SetInteger_type_support_ids_t _SetInteger_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _SetInteger_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _SetInteger_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _SetInteger_type_support_symbol_names_t _SetInteger_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, rc110_msgs, srv, SetInteger)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, rc110_msgs, srv, SetInteger)),
  }
};

typedef struct _SetInteger_type_support_data_t
{
  void * data[2];
} _SetInteger_type_support_data_t;

static _SetInteger_type_support_data_t _SetInteger_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _SetInteger_service_typesupport_map = {
  2,
  "rc110_msgs",
  &_SetInteger_service_typesupport_ids.typesupport_identifier[0],
  &_SetInteger_service_typesupport_symbol_names.symbol_name[0],
  &_SetInteger_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t SetInteger_service_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_SetInteger_service_typesupport_map),
  ::rosidl_typesupport_cpp::get_service_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace rc110_msgs

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
get_service_type_support_handle<rc110_msgs::srv::SetInteger>()
{
  return &::rc110_msgs::srv::rosidl_typesupport_cpp::SetInteger_service_type_support_handle;
}

}  // namespace rosidl_typesupport_cpp
