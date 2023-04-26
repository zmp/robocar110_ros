// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from rc110_msgs:msg/WheelSpeeds.idl
// generated code does not contain a copyright notice
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <Python.h>
#include <stdbool.h>
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-function"
#endif
#include "numpy/ndarrayobject.h"
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif
#include "rosidl_runtime_c/visibility_control.h"
#include "rc110_msgs/msg/detail/wheel_speeds__struct.h"
#include "rc110_msgs/msg/detail/wheel_speeds__functions.h"

ROSIDL_GENERATOR_C_IMPORT
bool std_msgs__msg__header__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * std_msgs__msg__header__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool rc110_msgs__msg__wheel_speeds__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[41];
    {
      char * class_name = NULL;
      char * module_name = NULL;
      {
        PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
        if (class_attr) {
          PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
          if (name_attr) {
            class_name = (char *)PyUnicode_1BYTE_DATA(name_attr);
            Py_DECREF(name_attr);
          }
          PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
          if (module_attr) {
            module_name = (char *)PyUnicode_1BYTE_DATA(module_attr);
            Py_DECREF(module_attr);
          }
          Py_DECREF(class_attr);
        }
      }
      if (!class_name || !module_name) {
        return false;
      }
      snprintf(full_classname_dest, sizeof(full_classname_dest), "%s.%s", module_name, class_name);
    }
    assert(strncmp("rc110_msgs.msg._wheel_speeds.WheelSpeeds", full_classname_dest, 40) == 0);
  }
  rc110_msgs__msg__WheelSpeeds * ros_message = _ros_message;
  {  // header
    PyObject * field = PyObject_GetAttrString(_pymsg, "header");
    if (!field) {
      return false;
    }
    if (!std_msgs__msg__header__convert_from_py(field, &ros_message->header)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // speed_fl
    PyObject * field = PyObject_GetAttrString(_pymsg, "speed_fl");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->speed_fl = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // speed_fr
    PyObject * field = PyObject_GetAttrString(_pymsg, "speed_fr");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->speed_fr = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // speed_rl
    PyObject * field = PyObject_GetAttrString(_pymsg, "speed_rl");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->speed_rl = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // speed_rr
    PyObject * field = PyObject_GetAttrString(_pymsg, "speed_rr");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->speed_rr = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * rc110_msgs__msg__wheel_speeds__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of WheelSpeeds */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("rc110_msgs.msg._wheel_speeds");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "WheelSpeeds");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  rc110_msgs__msg__WheelSpeeds * ros_message = (rc110_msgs__msg__WheelSpeeds *)raw_ros_message;
  {  // header
    PyObject * field = NULL;
    field = std_msgs__msg__header__convert_to_py(&ros_message->header);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "header", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // speed_fl
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->speed_fl);
    {
      int rc = PyObject_SetAttrString(_pymessage, "speed_fl", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // speed_fr
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->speed_fr);
    {
      int rc = PyObject_SetAttrString(_pymessage, "speed_fr", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // speed_rl
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->speed_rl);
    {
      int rc = PyObject_SetAttrString(_pymessage, "speed_rl", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // speed_rr
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->speed_rr);
    {
      int rc = PyObject_SetAttrString(_pymessage, "speed_rr", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
