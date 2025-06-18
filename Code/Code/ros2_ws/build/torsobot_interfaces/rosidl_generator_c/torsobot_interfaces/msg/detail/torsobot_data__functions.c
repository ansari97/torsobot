// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from torsobot_interfaces:msg/TorsobotData.idl
// generated code does not contain a copyright notice
#include "torsobot_interfaces/msg/detail/torsobot_data__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
torsobot_interfaces__msg__TorsobotData__init(torsobot_interfaces__msg__TorsobotData * msg)
{
  if (!msg) {
    return false;
  }
  // torso_pitch
  // torso_pitch_rate
  // motor_pos
  // motor_vel
  // motor_torque
  return true;
}

void
torsobot_interfaces__msg__TorsobotData__fini(torsobot_interfaces__msg__TorsobotData * msg)
{
  if (!msg) {
    return;
  }
  // torso_pitch
  // torso_pitch_rate
  // motor_pos
  // motor_vel
  // motor_torque
}

bool
torsobot_interfaces__msg__TorsobotData__are_equal(const torsobot_interfaces__msg__TorsobotData * lhs, const torsobot_interfaces__msg__TorsobotData * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // torso_pitch
  if (lhs->torso_pitch != rhs->torso_pitch) {
    return false;
  }
  // torso_pitch_rate
  if (lhs->torso_pitch_rate != rhs->torso_pitch_rate) {
    return false;
  }
  // motor_pos
  if (lhs->motor_pos != rhs->motor_pos) {
    return false;
  }
  // motor_vel
  if (lhs->motor_vel != rhs->motor_vel) {
    return false;
  }
  // motor_torque
  if (lhs->motor_torque != rhs->motor_torque) {
    return false;
  }
  return true;
}

bool
torsobot_interfaces__msg__TorsobotData__copy(
  const torsobot_interfaces__msg__TorsobotData * input,
  torsobot_interfaces__msg__TorsobotData * output)
{
  if (!input || !output) {
    return false;
  }
  // torso_pitch
  output->torso_pitch = input->torso_pitch;
  // torso_pitch_rate
  output->torso_pitch_rate = input->torso_pitch_rate;
  // motor_pos
  output->motor_pos = input->motor_pos;
  // motor_vel
  output->motor_vel = input->motor_vel;
  // motor_torque
  output->motor_torque = input->motor_torque;
  return true;
}

torsobot_interfaces__msg__TorsobotData *
torsobot_interfaces__msg__TorsobotData__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  torsobot_interfaces__msg__TorsobotData * msg = (torsobot_interfaces__msg__TorsobotData *)allocator.allocate(sizeof(torsobot_interfaces__msg__TorsobotData), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(torsobot_interfaces__msg__TorsobotData));
  bool success = torsobot_interfaces__msg__TorsobotData__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
torsobot_interfaces__msg__TorsobotData__destroy(torsobot_interfaces__msg__TorsobotData * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    torsobot_interfaces__msg__TorsobotData__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
torsobot_interfaces__msg__TorsobotData__Sequence__init(torsobot_interfaces__msg__TorsobotData__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  torsobot_interfaces__msg__TorsobotData * data = NULL;

  if (size) {
    data = (torsobot_interfaces__msg__TorsobotData *)allocator.zero_allocate(size, sizeof(torsobot_interfaces__msg__TorsobotData), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = torsobot_interfaces__msg__TorsobotData__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        torsobot_interfaces__msg__TorsobotData__fini(&data[i - 1]);
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
torsobot_interfaces__msg__TorsobotData__Sequence__fini(torsobot_interfaces__msg__TorsobotData__Sequence * array)
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
      torsobot_interfaces__msg__TorsobotData__fini(&array->data[i]);
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

torsobot_interfaces__msg__TorsobotData__Sequence *
torsobot_interfaces__msg__TorsobotData__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  torsobot_interfaces__msg__TorsobotData__Sequence * array = (torsobot_interfaces__msg__TorsobotData__Sequence *)allocator.allocate(sizeof(torsobot_interfaces__msg__TorsobotData__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = torsobot_interfaces__msg__TorsobotData__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
torsobot_interfaces__msg__TorsobotData__Sequence__destroy(torsobot_interfaces__msg__TorsobotData__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    torsobot_interfaces__msg__TorsobotData__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
torsobot_interfaces__msg__TorsobotData__Sequence__are_equal(const torsobot_interfaces__msg__TorsobotData__Sequence * lhs, const torsobot_interfaces__msg__TorsobotData__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!torsobot_interfaces__msg__TorsobotData__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
torsobot_interfaces__msg__TorsobotData__Sequence__copy(
  const torsobot_interfaces__msg__TorsobotData__Sequence * input,
  torsobot_interfaces__msg__TorsobotData__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(torsobot_interfaces__msg__TorsobotData);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    torsobot_interfaces__msg__TorsobotData * data =
      (torsobot_interfaces__msg__TorsobotData *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!torsobot_interfaces__msg__TorsobotData__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          torsobot_interfaces__msg__TorsobotData__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!torsobot_interfaces__msg__TorsobotData__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
