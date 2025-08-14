// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from torsobot_interfaces:msg/TorsobotData.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "torsobot_interfaces/msg/torsobot_data.h"


#ifndef TORSOBOT_INTERFACES__MSG__DETAIL__TORSOBOT_DATA__FUNCTIONS_H_
#define TORSOBOT_INTERFACES__MSG__DETAIL__TORSOBOT_DATA__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/action_type_support_struct.h"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_runtime_c/service_type_support_struct.h"
#include "rosidl_runtime_c/type_description/type_description__struct.h"
#include "rosidl_runtime_c/type_description/type_source__struct.h"
#include "rosidl_runtime_c/type_hash.h"
#include "rosidl_runtime_c/visibility_control.h"
#include "torsobot_interfaces/msg/rosidl_generator_c__visibility_control.h"

#include "torsobot_interfaces/msg/detail/torsobot_data__struct.h"

/// Initialize msg/TorsobotData message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * torsobot_interfaces__msg__TorsobotData
 * )) before or use
 * torsobot_interfaces__msg__TorsobotData__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_torsobot_interfaces
bool
torsobot_interfaces__msg__TorsobotData__init(torsobot_interfaces__msg__TorsobotData * msg);

/// Finalize msg/TorsobotData message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_torsobot_interfaces
void
torsobot_interfaces__msg__TorsobotData__fini(torsobot_interfaces__msg__TorsobotData * msg);

/// Create msg/TorsobotData message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * torsobot_interfaces__msg__TorsobotData__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_torsobot_interfaces
torsobot_interfaces__msg__TorsobotData *
torsobot_interfaces__msg__TorsobotData__create(void);

/// Destroy msg/TorsobotData message.
/**
 * It calls
 * torsobot_interfaces__msg__TorsobotData__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_torsobot_interfaces
void
torsobot_interfaces__msg__TorsobotData__destroy(torsobot_interfaces__msg__TorsobotData * msg);

/// Check for msg/TorsobotData message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_torsobot_interfaces
bool
torsobot_interfaces__msg__TorsobotData__are_equal(const torsobot_interfaces__msg__TorsobotData * lhs, const torsobot_interfaces__msg__TorsobotData * rhs);

/// Copy a msg/TorsobotData message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_torsobot_interfaces
bool
torsobot_interfaces__msg__TorsobotData__copy(
  const torsobot_interfaces__msg__TorsobotData * input,
  torsobot_interfaces__msg__TorsobotData * output);

/// Retrieve pointer to the hash of the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_torsobot_interfaces
const rosidl_type_hash_t *
torsobot_interfaces__msg__TorsobotData__get_type_hash(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_torsobot_interfaces
const rosidl_runtime_c__type_description__TypeDescription *
torsobot_interfaces__msg__TorsobotData__get_type_description(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the single raw source text that defined this type.
ROSIDL_GENERATOR_C_PUBLIC_torsobot_interfaces
const rosidl_runtime_c__type_description__TypeSource *
torsobot_interfaces__msg__TorsobotData__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the recursive raw sources that defined the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_torsobot_interfaces
const rosidl_runtime_c__type_description__TypeSource__Sequence *
torsobot_interfaces__msg__TorsobotData__get_type_description_sources(
  const rosidl_message_type_support_t * type_support);

/// Initialize array of msg/TorsobotData messages.
/**
 * It allocates the memory for the number of elements and calls
 * torsobot_interfaces__msg__TorsobotData__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_torsobot_interfaces
bool
torsobot_interfaces__msg__TorsobotData__Sequence__init(torsobot_interfaces__msg__TorsobotData__Sequence * array, size_t size);

/// Finalize array of msg/TorsobotData messages.
/**
 * It calls
 * torsobot_interfaces__msg__TorsobotData__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_torsobot_interfaces
void
torsobot_interfaces__msg__TorsobotData__Sequence__fini(torsobot_interfaces__msg__TorsobotData__Sequence * array);

/// Create array of msg/TorsobotData messages.
/**
 * It allocates the memory for the array and calls
 * torsobot_interfaces__msg__TorsobotData__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_torsobot_interfaces
torsobot_interfaces__msg__TorsobotData__Sequence *
torsobot_interfaces__msg__TorsobotData__Sequence__create(size_t size);

/// Destroy array of msg/TorsobotData messages.
/**
 * It calls
 * torsobot_interfaces__msg__TorsobotData__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_torsobot_interfaces
void
torsobot_interfaces__msg__TorsobotData__Sequence__destroy(torsobot_interfaces__msg__TorsobotData__Sequence * array);

/// Check for msg/TorsobotData message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_torsobot_interfaces
bool
torsobot_interfaces__msg__TorsobotData__Sequence__are_equal(const torsobot_interfaces__msg__TorsobotData__Sequence * lhs, const torsobot_interfaces__msg__TorsobotData__Sequence * rhs);

/// Copy an array of msg/TorsobotData messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_torsobot_interfaces
bool
torsobot_interfaces__msg__TorsobotData__Sequence__copy(
  const torsobot_interfaces__msg__TorsobotData__Sequence * input,
  torsobot_interfaces__msg__TorsobotData__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // TORSOBOT_INTERFACES__MSG__DETAIL__TORSOBOT_DATA__FUNCTIONS_H_
