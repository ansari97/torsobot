// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from torsobot_interfaces:msg/TorsobotData.idl
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
#include "torsobot_interfaces/msg/detail/torsobot_data__struct.h"
#include "torsobot_interfaces/msg/detail/torsobot_data__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool torsobot_interfaces__msg__torsobot_data__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[52];
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
    assert(strncmp("torsobot_interfaces.msg._torsobot_data.TorsobotData", full_classname_dest, 51) == 0);
  }
  torsobot_interfaces__msg__TorsobotData * ros_message = _ros_message;
  {  // torso_pitch
    PyObject * field = PyObject_GetAttrString(_pymsg, "torso_pitch");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->torso_pitch = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // torso_pitch_rate
    PyObject * field = PyObject_GetAttrString(_pymsg, "torso_pitch_rate");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->torso_pitch_rate = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // motor_pos
    PyObject * field = PyObject_GetAttrString(_pymsg, "motor_pos");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->motor_pos = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // motor_vel
    PyObject * field = PyObject_GetAttrString(_pymsg, "motor_vel");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->motor_vel = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // motor_torque
    PyObject * field = PyObject_GetAttrString(_pymsg, "motor_torque");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->motor_torque = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // motor_drv_mode
    PyObject * field = PyObject_GetAttrString(_pymsg, "motor_drv_mode");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->motor_drv_mode = (int8_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * torsobot_interfaces__msg__torsobot_data__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of TorsobotData */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("torsobot_interfaces.msg._torsobot_data");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "TorsobotData");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  torsobot_interfaces__msg__TorsobotData * ros_message = (torsobot_interfaces__msg__TorsobotData *)raw_ros_message;
  {  // torso_pitch
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->torso_pitch);
    {
      int rc = PyObject_SetAttrString(_pymessage, "torso_pitch", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // torso_pitch_rate
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->torso_pitch_rate);
    {
      int rc = PyObject_SetAttrString(_pymessage, "torso_pitch_rate", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // motor_pos
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->motor_pos);
    {
      int rc = PyObject_SetAttrString(_pymessage, "motor_pos", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // motor_vel
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->motor_vel);
    {
      int rc = PyObject_SetAttrString(_pymessage, "motor_vel", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // motor_torque
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->motor_torque);
    {
      int rc = PyObject_SetAttrString(_pymessage, "motor_torque", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // motor_drv_mode
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->motor_drv_mode);
    {
      int rc = PyObject_SetAttrString(_pymessage, "motor_drv_mode", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
