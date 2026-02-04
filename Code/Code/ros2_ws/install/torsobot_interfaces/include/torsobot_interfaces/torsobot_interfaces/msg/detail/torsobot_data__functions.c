// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from torsobot_interfaces:msg/TorsobotData.idl
// generated code does not contain a copyright notice
#include "torsobot_interfaces/msg/detail/torsobot_data__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `torsobot_state`
#include "torsobot_interfaces/msg/detail/torsobot_state__functions.h"

bool
torsobot_interfaces__msg__TorsobotData__init(torsobot_interfaces__msg__TorsobotData * msg)
{
  if (!msg) {
    return false;
  }
  // torsobot_state
  if (!torsobot_interfaces__msg__TorsobotState__init(&msg->torsobot_state)) {
    torsobot_interfaces__msg__TorsobotData__fini(msg);
    return false;
  }
  // wheel_torque
  // wheel_cmd_torque
  // mot_drv_mode
  // torso_pitch_init
  return true;
}

void
torsobot_interfaces__msg__TorsobotData__fini(torsobot_interfaces__msg__TorsobotData * msg)
{
  if (!msg) {
    return;
  }
  // torsobot_state
  torsobot_interfaces__msg__TorsobotState__fini(&msg->torsobot_state);
  // wheel_torque
  // wheel_cmd_torque
  // mot_drv_mode
  // torso_pitch_init
}

bool
torsobot_interfaces__msg__TorsobotData__are_equal(const torsobot_interfaces__msg__TorsobotData * lhs, const torsobot_interfaces__msg__TorsobotData * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // torsobot_state
  if (!torsobot_interfaces__msg__TorsobotState__are_equal(
      &(lhs->torsobot_state), &(rhs->torsobot_state)))
  {
    return false;
  }
  // wheel_torque
  if (lhs->wheel_torque != rhs->wheel_torque) {
    return false;
  }
  // wheel_cmd_torque
  if (lhs->wheel_cmd_torque != rhs->wheel_cmd_torque) {
    return false;
  }
  // mot_drv_mode
  if (lhs->mot_drv_mode != rhs->mot_drv_mode) {
    return false;
  }
  // torso_pitch_init
  if (lhs->torso_pitch_init != rhs->torso_pitch_init) {
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
  // torsobot_state
  if (!torsobot_interfaces__msg__TorsobotState__copy(
      &(input->torsobot_state), &(output->torsobot_state)))
  {
    return false;
  }
  // wheel_torque
  output->wheel_torque = input->wheel_torque;
  // wheel_cmd_torque
  output->wheel_cmd_torque = input->wheel_cmd_torque;
  // mot_drv_mode
  output->mot_drv_mode = input->mot_drv_mode;
  // torso_pitch_init
  output->torso_pitch_init = input->torso_pitch_init;
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
