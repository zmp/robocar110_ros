// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from rc110_msgs:msg/Status.idl
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
#include "rc110_msgs/msg/detail/status__struct.h"
#include "rc110_msgs/msg/detail/status__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool rc110_msgs__msg__status__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[30];
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
    assert(strncmp("rc110_msgs.msg._status.Status", full_classname_dest, 29) == 0);
  }
  rc110_msgs__msg__Status * ros_message = _ros_message;
  {  // board_enabled
    PyObject * field = PyObject_GetAttrString(_pymsg, "board_enabled");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->board_enabled = (Py_True == field);
    Py_DECREF(field);
  }
  {  // motor_state
    PyObject * field = PyObject_GetAttrString(_pymsg, "motor_state");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->motor_state = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // servo_state
    PyObject * field = PyObject_GetAttrString(_pymsg, "servo_state");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->servo_state = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * rc110_msgs__msg__status__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of Status */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("rc110_msgs.msg._status");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "Status");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  rc110_msgs__msg__Status * ros_message = (rc110_msgs__msg__Status *)raw_ros_message;
  {  // board_enabled
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->board_enabled ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "board_enabled", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // motor_state
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->motor_state);
    {
      int rc = PyObject_SetAttrString(_pymessage, "motor_state", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // servo_state
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->servo_state);
    {
      int rc = PyObject_SetAttrString(_pymessage, "servo_state", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
