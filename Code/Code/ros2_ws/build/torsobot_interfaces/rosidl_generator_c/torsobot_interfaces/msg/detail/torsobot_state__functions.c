// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from torsobot_interfaces:msg/TorsobotState.idl
// generated code does not contain a copyright notice
#include "torsobot_interfaces/msg/detail/torsobot_state__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
torsobot_interfaces__msg__TorsobotState__init(torsobot_interfaces__msg__TorsobotState * msg)
{
  if (!msg) {
    return false;
  }
  // torso_pitch
  // torso_pitch_rate
  // wheel_pos
  // wheel_vel
  return true;
}

void
torsobot_interfaces__msg__TorsobotState__fini(torsobot_interfaces__msg__TorsobotState * msg)
{
  if (!msg) {
    return;
  }
  // torso_pitch
  // torso_pitch_rate
  // wheel_pos
  // wheel_vel
}

bool
torsobot_interfaces__msg__TorsobotState__are_equal(const torsobot_interfaces__msg__TorsobotState * lhs, const torsobot_interfaces__msg__TorsobotState * rhs)
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
  // wheel_pos
  if (lhs->wheel_pos != rhs->wheel_pos) {
    return false;
  }
  // wheel_vel
  if (lhs->wheel_vel != rhs->wheel_vel) {
    return false;
  }
  return true;
}

bool
torsobot_interfaces__msg__TorsobotState__copy(
  const torsobot_interfaces__msg__TorsobotState * input,
  torsobot_interfaces__msg__TorsobotState * output)
{
  if (!input || !output) {
    return false;
  }
  // torso_pitch
  output->torso_pitch = input->torso_pitch;
  // torso_pitch_rate
  output->torso_pitch_rate = input->torso_pitch_rate;
  // wheel_pos
  output->wheel_pos = input->wheel_pos;
  // wheel_vel
  output->wheel_vel = input->wheel_vel;
  return true;
}

torsobot_interfaces__msg__TorsobotState *
torsobot_interfaces__msg__TorsobotState__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  torsobot_interfaces__msg__TorsobotState * msg = (torsobot_interfaces__msg__TorsobotState *)allocator.allocate(sizeof(torsobot_interfaces__msg__TorsobotState), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(torsobot_interfaces__msg__TorsobotState));
  bool success = torsobot_interfaces__msg__TorsobotState__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
torsobot_interfaces__msg__TorsobotState__destroy(torsobot_interfaces__msg__TorsobotState * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    torsobot_interfaces__msg__TorsobotState__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
torsobot_interfaces__msg__TorsobotState__Sequence__init(torsobot_interfaces__msg__TorsobotState__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  torsobot_interfaces__msg__TorsobotState * data = NULL;

  if (size) {
    data = (torsobot_interfaces__msg__TorsobotState *)allocator.zero_allocate(size, sizeof(torsobot_interfaces__msg__TorsobotState), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = torsobot_interfaces__msg__TorsobotState__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        torsobot_interfaces__msg__TorsobotState__fini(&data[i - 1]);
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
torsobot_interfaces__msg__TorsobotState__Sequence__fini(torsobot_interfaces__msg__TorsobotState__Sequence * array)
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
      torsobot_interfaces__msg__TorsobotState__fini(&array->data[i]);
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

torsobot_interfaces__msg__TorsobotState__Sequence *
torsobot_interfaces__msg__TorsobotState__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  torsobot_interfaces__msg__TorsobotState__Sequence * array = (torsobot_interfaces__msg__TorsobotState__Sequence *)allocator.allocate(sizeof(torsobot_interfaces__msg__TorsobotState__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = torsobot_interfaces__msg__TorsobotState__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
torsobot_interfaces__msg__TorsobotState__Sequence__destroy(torsobot_interfaces__msg__TorsobotState__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    torsobot_interfaces__msg__TorsobotState__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
torsobot_interfaces__msg__TorsobotState__Sequence__are_equal(const torsobot_interfaces__msg__TorsobotState__Sequence * lhs, const torsobot_interfaces__msg__TorsobotState__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!torsobot_interfaces__msg__TorsobotState__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
torsobot_interfaces__msg__TorsobotState__Sequence__copy(
  const torsobot_interfaces__msg__TorsobotState__Sequence * input,
  torsobot_interfaces__msg__TorsobotState__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(torsobot_interfaces__msg__TorsobotState);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    torsobot_interfaces__msg__TorsobotState * data =
      (torsobot_interfaces__msg__TorsobotState *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!torsobot_interfaces__msg__TorsobotState__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          torsobot_interfaces__msg__TorsobotState__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!torsobot_interfaces__msg__TorsobotState__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
