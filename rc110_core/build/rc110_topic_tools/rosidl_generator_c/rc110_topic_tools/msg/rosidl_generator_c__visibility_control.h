// generated from rosidl_generator_c/resource/rosidl_generator_c__visibility_control.h.in
// generated code does not contain a copyright notice

#ifndef RC110_TOPIC_TOOLS__MSG__ROSIDL_GENERATOR_C__VISIBILITY_CONTROL_H_
#define RC110_TOPIC_TOOLS__MSG__ROSIDL_GENERATOR_C__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ROSIDL_GENERATOR_C_EXPORT_rc110_topic_tools __attribute__ ((dllexport))
    #define ROSIDL_GENERATOR_C_IMPORT_rc110_topic_tools __attribute__ ((dllimport))
  #else
    #define ROSIDL_GENERATOR_C_EXPORT_rc110_topic_tools __declspec(dllexport)
    #define ROSIDL_GENERATOR_C_IMPORT_rc110_topic_tools __declspec(dllimport)
  #endif
  #ifdef ROSIDL_GENERATOR_C_BUILDING_DLL_rc110_topic_tools
    #define ROSIDL_GENERATOR_C_PUBLIC_rc110_topic_tools ROSIDL_GENERATOR_C_EXPORT_rc110_topic_tools
  #else
    #define ROSIDL_GENERATOR_C_PUBLIC_rc110_topic_tools ROSIDL_GENERATOR_C_IMPORT_rc110_topic_tools
  #endif
#else
  #define ROSIDL_GENERATOR_C_EXPORT_rc110_topic_tools __attribute__ ((visibility("default")))
  #define ROSIDL_GENERATOR_C_IMPORT_rc110_topic_tools
  #if __GNUC__ >= 4
    #define ROSIDL_GENERATOR_C_PUBLIC_rc110_topic_tools __attribute__ ((visibility("default")))
  #else
    #define ROSIDL_GENERATOR_C_PUBLIC_rc110_topic_tools
  #endif
#endif

#ifdef __cplusplus
}
#endif

#endif  // RC110_TOPIC_TOOLS__MSG__ROSIDL_GENERATOR_C__VISIBILITY_CONTROL_H_
