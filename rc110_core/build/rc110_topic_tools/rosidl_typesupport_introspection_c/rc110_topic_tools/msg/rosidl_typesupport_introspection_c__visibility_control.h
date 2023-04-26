// generated from
// rosidl_typesupport_introspection_c/resource/rosidl_typesupport_introspection_c__visibility_control.h.in
// generated code does not contain a copyright notice

#ifndef RC110_TOPIC_TOOLS__MSG__ROSIDL_TYPESUPPORT_INTROSPECTION_C__VISIBILITY_CONTROL_H_
#define RC110_TOPIC_TOOLS__MSG__ROSIDL_TYPESUPPORT_INTROSPECTION_C__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_rc110_topic_tools __attribute__ ((dllexport))
    #define ROSIDL_TYPESUPPORT_INTROSPECTION_C_IMPORT_rc110_topic_tools __attribute__ ((dllimport))
  #else
    #define ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_rc110_topic_tools __declspec(dllexport)
    #define ROSIDL_TYPESUPPORT_INTROSPECTION_C_IMPORT_rc110_topic_tools __declspec(dllimport)
  #endif
  #ifdef ROSIDL_TYPESUPPORT_INTROSPECTION_C_BUILDING_DLL_rc110_topic_tools
    #define ROSIDL_TYPESUPPORT_INTROSPECTION_C_PUBLIC_rc110_topic_tools ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_rc110_topic_tools
  #else
    #define ROSIDL_TYPESUPPORT_INTROSPECTION_C_PUBLIC_rc110_topic_tools ROSIDL_TYPESUPPORT_INTROSPECTION_C_IMPORT_rc110_topic_tools
  #endif
#else
  #define ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_rc110_topic_tools __attribute__ ((visibility("default")))
  #define ROSIDL_TYPESUPPORT_INTROSPECTION_C_IMPORT_rc110_topic_tools
  #if __GNUC__ >= 4
    #define ROSIDL_TYPESUPPORT_INTROSPECTION_C_PUBLIC_rc110_topic_tools __attribute__ ((visibility("default")))
  #else
    #define ROSIDL_TYPESUPPORT_INTROSPECTION_C_PUBLIC_rc110_topic_tools
  #endif
#endif

#ifdef __cplusplus
}
#endif

#endif  // RC110_TOPIC_TOOLS__MSG__ROSIDL_TYPESUPPORT_INTROSPECTION_C__VISIBILITY_CONTROL_H_
