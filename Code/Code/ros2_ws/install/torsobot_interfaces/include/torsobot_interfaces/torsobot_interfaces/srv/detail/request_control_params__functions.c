// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from torsobot_interfaces:srv/RequestControlParams.idl
// generated code does not contain a copyright notice
#include "torsobot_interfaces/srv/detail/request_control_params__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

bool
torsobot_interfaces__srv__RequestControlParams_Request__init(torsobot_interfaces__srv__RequestControlParams_Request * msg)
{
  if (!msg) {
    return false;
  }
  // structure_needs_at_least_one_member
  return true;
}

void
torsobot_interfaces__srv__RequestControlParams_Request__fini(torsobot_interfaces__srv__RequestControlParams_Request * msg)
{
  if (!msg) {
    return;
  }
  // structure_needs_at_least_one_member
}

bool
torsobot_interfaces__srv__RequestControlParams_Request__are_equal(const torsobot_interfaces__srv__RequestControlParams_Request * lhs, const torsobot_interfaces__srv__RequestControlParams_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // structure_needs_at_least_one_member
  if (lhs->structure_needs_at_least_one_member != rhs->structure_needs_at_least_one_member) {
    return false;
  }
  return true;
}

bool
torsobot_interfaces__srv__RequestControlParams_Request__copy(
  const torsobot_interfaces__srv__RequestControlParams_Request * input,
  torsobot_interfaces__srv__RequestControlParams_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // structure_needs_at_least_one_member
  output->structure_needs_at_least_one_member = input->structure_needs_at_least_one_member;
  return true;
}

torsobot_interfaces__srv__RequestControlParams_Request *
torsobot_interfaces__srv__RequestControlParams_Request__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  torsobot_interfaces__srv__RequestControlParams_Request * msg = (torsobot_interfaces__srv__RequestControlParams_Request *)allocator.allocate(sizeof(torsobot_interfaces__srv__RequestControlParams_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(torsobot_interfaces__srv__RequestControlParams_Request));
  bool success = torsobot_interfaces__srv__RequestControlParams_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
torsobot_interfaces__srv__RequestControlParams_Request__destroy(torsobot_interfaces__srv__RequestControlParams_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    torsobot_interfaces__srv__RequestControlParams_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
torsobot_interfaces__srv__RequestControlParams_Request__Sequence__init(torsobot_interfaces__srv__RequestControlParams_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  torsobot_interfaces__srv__RequestControlParams_Request * data = NULL;

  if (size) {
    data = (torsobot_interfaces__srv__RequestControlParams_Request *)allocator.zero_allocate(size, sizeof(torsobot_interfaces__srv__RequestControlParams_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = torsobot_interfaces__srv__RequestControlParams_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        torsobot_interfaces__srv__RequestControlParams_Request__fini(&data[i - 1]);
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
torsobot_interfaces__srv__RequestControlParams_Request__Sequence__fini(torsobot_interfaces__srv__RequestControlParams_Request__Sequence * array)
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
      torsobot_interfaces__srv__RequestControlParams_Request__fini(&array->data[i]);
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

torsobot_interfaces__srv__RequestControlParams_Request__Sequence *
torsobot_interfaces__srv__RequestControlParams_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  torsobot_interfaces__srv__RequestControlParams_Request__Sequence * array = (torsobot_interfaces__srv__RequestControlParams_Request__Sequence *)allocator.allocate(sizeof(torsobot_interfaces__srv__RequestControlParams_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = torsobot_interfaces__srv__RequestControlParams_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
torsobot_interfaces__srv__RequestControlParams_Request__Sequence__destroy(torsobot_interfaces__srv__RequestControlParams_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    torsobot_interfaces__srv__RequestControlParams_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
torsobot_interfaces__srv__RequestControlParams_Request__Sequence__are_equal(const torsobot_interfaces__srv__RequestControlParams_Request__Sequence * lhs, const torsobot_interfaces__srv__RequestControlParams_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!torsobot_interfaces__srv__RequestControlParams_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
torsobot_interfaces__srv__RequestControlParams_Request__Sequence__copy(
  const torsobot_interfaces__srv__RequestControlParams_Request__Sequence * input,
  torsobot_interfaces__srv__RequestControlParams_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(torsobot_interfaces__srv__RequestControlParams_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    torsobot_interfaces__srv__RequestControlParams_Request * data =
      (torsobot_interfaces__srv__RequestControlParams_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!torsobot_interfaces__srv__RequestControlParams_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          torsobot_interfaces__srv__RequestControlParams_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!torsobot_interfaces__srv__RequestControlParams_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


bool
torsobot_interfaces__srv__RequestControlParams_Response__init(torsobot_interfaces__srv__RequestControlParams_Response * msg)
{
  if (!msg) {
    return false;
  }
  // desired_torso_pitch
  // mot_max_torque
  // kp
  // ki
  // kd
  return true;
}

void
torsobot_interfaces__srv__RequestControlParams_Response__fini(torsobot_interfaces__srv__RequestControlParams_Response * msg)
{
  if (!msg) {
    return;
  }
  // desired_torso_pitch
  // mot_max_torque
  // kp
  // ki
  // kd
}

bool
torsobot_interfaces__srv__RequestControlParams_Response__are_equal(const torsobot_interfaces__srv__RequestControlParams_Response * lhs, const torsobot_interfaces__srv__RequestControlParams_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // desired_torso_pitch
  if (lhs->desired_torso_pitch != rhs->desired_torso_pitch) {
    return false;
  }
  // mot_max_torque
  if (lhs->mot_max_torque != rhs->mot_max_torque) {
    return false;
  }
  // kp
  if (lhs->kp != rhs->kp) {
    return false;
  }
  // ki
  if (lhs->ki != rhs->ki) {
    return false;
  }
  // kd
  if (lhs->kd != rhs->kd) {
    return false;
  }
  return true;
}

bool
torsobot_interfaces__srv__RequestControlParams_Response__copy(
  const torsobot_interfaces__srv__RequestControlParams_Response * input,
  torsobot_interfaces__srv__RequestControlParams_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // desired_torso_pitch
  output->desired_torso_pitch = input->desired_torso_pitch;
  // mot_max_torque
  output->mot_max_torque = input->mot_max_torque;
  // kp
  output->kp = input->kp;
  // ki
  output->ki = input->ki;
  // kd
  output->kd = input->kd;
  return true;
}

torsobot_interfaces__srv__RequestControlParams_Response *
torsobot_interfaces__srv__RequestControlParams_Response__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  torsobot_interfaces__srv__RequestControlParams_Response * msg = (torsobot_interfaces__srv__RequestControlParams_Response *)allocator.allocate(sizeof(torsobot_interfaces__srv__RequestControlParams_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(torsobot_interfaces__srv__RequestControlParams_Response));
  bool success = torsobot_interfaces__srv__RequestControlParams_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
torsobot_interfaces__srv__RequestControlParams_Response__destroy(torsobot_interfaces__srv__RequestControlParams_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    torsobot_interfaces__srv__RequestControlParams_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
torsobot_interfaces__srv__RequestControlParams_Response__Sequence__init(torsobot_interfaces__srv__RequestControlParams_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  torsobot_interfaces__srv__RequestControlParams_Response * data = NULL;

  if (size) {
    data = (torsobot_interfaces__srv__RequestControlParams_Response *)allocator.zero_allocate(size, sizeof(torsobot_interfaces__srv__RequestControlParams_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = torsobot_interfaces__srv__RequestControlParams_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        torsobot_interfaces__srv__RequestControlParams_Response__fini(&data[i - 1]);
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
torsobot_interfaces__srv__RequestControlParams_Response__Sequence__fini(torsobot_interfaces__srv__RequestControlParams_Response__Sequence * array)
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
      torsobot_interfaces__srv__RequestControlParams_Response__fini(&array->data[i]);
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

torsobot_interfaces__srv__RequestControlParams_Response__Sequence *
torsobot_interfaces__srv__RequestControlParams_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  torsobot_interfaces__srv__RequestControlParams_Response__Sequence * array = (torsobot_interfaces__srv__RequestControlParams_Response__Sequence *)allocator.allocate(sizeof(torsobot_interfaces__srv__RequestControlParams_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = torsobot_interfaces__srv__RequestControlParams_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
torsobot_interfaces__srv__RequestControlParams_Response__Sequence__destroy(torsobot_interfaces__srv__RequestControlParams_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    torsobot_interfaces__srv__RequestControlParams_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
torsobot_interfaces__srv__RequestControlParams_Response__Sequence__are_equal(const torsobot_interfaces__srv__RequestControlParams_Response__Sequence * lhs, const torsobot_interfaces__srv__RequestControlParams_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!torsobot_interfaces__srv__RequestControlParams_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
torsobot_interfaces__srv__RequestControlParams_Response__Sequence__copy(
  const torsobot_interfaces__srv__RequestControlParams_Response__Sequence * input,
  torsobot_interfaces__srv__RequestControlParams_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(torsobot_interfaces__srv__RequestControlParams_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    torsobot_interfaces__srv__RequestControlParams_Response * data =
      (torsobot_interfaces__srv__RequestControlParams_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!torsobot_interfaces__srv__RequestControlParams_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          torsobot_interfaces__srv__RequestControlParams_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!torsobot_interfaces__srv__RequestControlParams_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `info`
#include "service_msgs/msg/detail/service_event_info__functions.h"
// Member `request`
// Member `response`
// already included above
// #include "torsobot_interfaces/srv/detail/request_control_params__functions.h"

bool
torsobot_interfaces__srv__RequestControlParams_Event__init(torsobot_interfaces__srv__RequestControlParams_Event * msg)
{
  if (!msg) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__init(&msg->info)) {
    torsobot_interfaces__srv__RequestControlParams_Event__fini(msg);
    return false;
  }
  // request
  if (!torsobot_interfaces__srv__RequestControlParams_Request__Sequence__init(&msg->request, 0)) {
    torsobot_interfaces__srv__RequestControlParams_Event__fini(msg);
    return false;
  }
  // response
  if (!torsobot_interfaces__srv__RequestControlParams_Response__Sequence__init(&msg->response, 0)) {
    torsobot_interfaces__srv__RequestControlParams_Event__fini(msg);
    return false;
  }
  return true;
}

void
torsobot_interfaces__srv__RequestControlParams_Event__fini(torsobot_interfaces__srv__RequestControlParams_Event * msg)
{
  if (!msg) {
    return;
  }
  // info
  service_msgs__msg__ServiceEventInfo__fini(&msg->info);
  // request
  torsobot_interfaces__srv__RequestControlParams_Request__Sequence__fini(&msg->request);
  // response
  torsobot_interfaces__srv__RequestControlParams_Response__Sequence__fini(&msg->response);
}

bool
torsobot_interfaces__srv__RequestControlParams_Event__are_equal(const torsobot_interfaces__srv__RequestControlParams_Event * lhs, const torsobot_interfaces__srv__RequestControlParams_Event * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__are_equal(
      &(lhs->info), &(rhs->info)))
  {
    return false;
  }
  // request
  if (!torsobot_interfaces__srv__RequestControlParams_Request__Sequence__are_equal(
      &(lhs->request), &(rhs->request)))
  {
    return false;
  }
  // response
  if (!torsobot_interfaces__srv__RequestControlParams_Response__Sequence__are_equal(
      &(lhs->response), &(rhs->response)))
  {
    return false;
  }
  return true;
}

bool
torsobot_interfaces__srv__RequestControlParams_Event__copy(
  const torsobot_interfaces__srv__RequestControlParams_Event * input,
  torsobot_interfaces__srv__RequestControlParams_Event * output)
{
  if (!input || !output) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__copy(
      &(input->info), &(output->info)))
  {
    return false;
  }
  // request
  if (!torsobot_interfaces__srv__RequestControlParams_Request__Sequence__copy(
      &(input->request), &(output->request)))
  {
    return false;
  }
  // response
  if (!torsobot_interfaces__srv__RequestControlParams_Response__Sequence__copy(
      &(input->response), &(output->response)))
  {
    return false;
  }
  return true;
}

torsobot_interfaces__srv__RequestControlParams_Event *
torsobot_interfaces__srv__RequestControlParams_Event__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  torsobot_interfaces__srv__RequestControlParams_Event * msg = (torsobot_interfaces__srv__RequestControlParams_Event *)allocator.allocate(sizeof(torsobot_interfaces__srv__RequestControlParams_Event), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(torsobot_interfaces__srv__RequestControlParams_Event));
  bool success = torsobot_interfaces__srv__RequestControlParams_Event__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
torsobot_interfaces__srv__RequestControlParams_Event__destroy(torsobot_interfaces__srv__RequestControlParams_Event * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    torsobot_interfaces__srv__RequestControlParams_Event__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
torsobot_interfaces__srv__RequestControlParams_Event__Sequence__init(torsobot_interfaces__srv__RequestControlParams_Event__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  torsobot_interfaces__srv__RequestControlParams_Event * data = NULL;

  if (size) {
    data = (torsobot_interfaces__srv__RequestControlParams_Event *)allocator.zero_allocate(size, sizeof(torsobot_interfaces__srv__RequestControlParams_Event), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = torsobot_interfaces__srv__RequestControlParams_Event__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        torsobot_interfaces__srv__RequestControlParams_Event__fini(&data[i - 1]);
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
torsobot_interfaces__srv__RequestControlParams_Event__Sequence__fini(torsobot_interfaces__srv__RequestControlParams_Event__Sequence * array)
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
      torsobot_interfaces__srv__RequestControlParams_Event__fini(&array->data[i]);
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

torsobot_interfaces__srv__RequestControlParams_Event__Sequence *
torsobot_interfaces__srv__RequestControlParams_Event__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  torsobot_interfaces__srv__RequestControlParams_Event__Sequence * array = (torsobot_interfaces__srv__RequestControlParams_Event__Sequence *)allocator.allocate(sizeof(torsobot_interfaces__srv__RequestControlParams_Event__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = torsobot_interfaces__srv__RequestControlParams_Event__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
torsobot_interfaces__srv__RequestControlParams_Event__Sequence__destroy(torsobot_interfaces__srv__RequestControlParams_Event__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    torsobot_interfaces__srv__RequestControlParams_Event__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
torsobot_interfaces__srv__RequestControlParams_Event__Sequence__are_equal(const torsobot_interfaces__srv__RequestControlParams_Event__Sequence * lhs, const torsobot_interfaces__srv__RequestControlParams_Event__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!torsobot_interfaces__srv__RequestControlParams_Event__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
torsobot_interfaces__srv__RequestControlParams_Event__Sequence__copy(
  const torsobot_interfaces__srv__RequestControlParams_Event__Sequence * input,
  torsobot_interfaces__srv__RequestControlParams_Event__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(torsobot_interfaces__srv__RequestControlParams_Event);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    torsobot_interfaces__srv__RequestControlParams_Event * data =
      (torsobot_interfaces__srv__RequestControlParams_Event *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!torsobot_interfaces__srv__RequestControlParams_Event__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          torsobot_interfaces__srv__RequestControlParams_Event__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!torsobot_interfaces__srv__RequestControlParams_Event__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
