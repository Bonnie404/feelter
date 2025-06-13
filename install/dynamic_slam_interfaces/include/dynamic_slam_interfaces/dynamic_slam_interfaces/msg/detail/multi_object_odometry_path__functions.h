// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from dynamic_slam_interfaces:msg/MultiObjectOdometryPath.idl
// generated code does not contain a copyright notice

#ifndef DYNAMIC_SLAM_INTERFACES__MSG__DETAIL__MULTI_OBJECT_ODOMETRY_PATH__FUNCTIONS_H_
#define DYNAMIC_SLAM_INTERFACES__MSG__DETAIL__MULTI_OBJECT_ODOMETRY_PATH__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "dynamic_slam_interfaces/msg/rosidl_generator_c__visibility_control.h"

#include "dynamic_slam_interfaces/msg/detail/multi_object_odometry_path__struct.h"

/// Initialize msg/MultiObjectOdometryPath message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * dynamic_slam_interfaces__msg__MultiObjectOdometryPath
 * )) before or use
 * dynamic_slam_interfaces__msg__MultiObjectOdometryPath__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_dynamic_slam_interfaces
bool
dynamic_slam_interfaces__msg__MultiObjectOdometryPath__init(dynamic_slam_interfaces__msg__MultiObjectOdometryPath * msg);

/// Finalize msg/MultiObjectOdometryPath message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_dynamic_slam_interfaces
void
dynamic_slam_interfaces__msg__MultiObjectOdometryPath__fini(dynamic_slam_interfaces__msg__MultiObjectOdometryPath * msg);

/// Create msg/MultiObjectOdometryPath message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * dynamic_slam_interfaces__msg__MultiObjectOdometryPath__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_dynamic_slam_interfaces
dynamic_slam_interfaces__msg__MultiObjectOdometryPath *
dynamic_slam_interfaces__msg__MultiObjectOdometryPath__create();

/// Destroy msg/MultiObjectOdometryPath message.
/**
 * It calls
 * dynamic_slam_interfaces__msg__MultiObjectOdometryPath__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_dynamic_slam_interfaces
void
dynamic_slam_interfaces__msg__MultiObjectOdometryPath__destroy(dynamic_slam_interfaces__msg__MultiObjectOdometryPath * msg);

/// Check for msg/MultiObjectOdometryPath message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_dynamic_slam_interfaces
bool
dynamic_slam_interfaces__msg__MultiObjectOdometryPath__are_equal(const dynamic_slam_interfaces__msg__MultiObjectOdometryPath * lhs, const dynamic_slam_interfaces__msg__MultiObjectOdometryPath * rhs);

/// Copy a msg/MultiObjectOdometryPath message.
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
ROSIDL_GENERATOR_C_PUBLIC_dynamic_slam_interfaces
bool
dynamic_slam_interfaces__msg__MultiObjectOdometryPath__copy(
  const dynamic_slam_interfaces__msg__MultiObjectOdometryPath * input,
  dynamic_slam_interfaces__msg__MultiObjectOdometryPath * output);

/// Initialize array of msg/MultiObjectOdometryPath messages.
/**
 * It allocates the memory for the number of elements and calls
 * dynamic_slam_interfaces__msg__MultiObjectOdometryPath__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_dynamic_slam_interfaces
bool
dynamic_slam_interfaces__msg__MultiObjectOdometryPath__Sequence__init(dynamic_slam_interfaces__msg__MultiObjectOdometryPath__Sequence * array, size_t size);

/// Finalize array of msg/MultiObjectOdometryPath messages.
/**
 * It calls
 * dynamic_slam_interfaces__msg__MultiObjectOdometryPath__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_dynamic_slam_interfaces
void
dynamic_slam_interfaces__msg__MultiObjectOdometryPath__Sequence__fini(dynamic_slam_interfaces__msg__MultiObjectOdometryPath__Sequence * array);

/// Create array of msg/MultiObjectOdometryPath messages.
/**
 * It allocates the memory for the array and calls
 * dynamic_slam_interfaces__msg__MultiObjectOdometryPath__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_dynamic_slam_interfaces
dynamic_slam_interfaces__msg__MultiObjectOdometryPath__Sequence *
dynamic_slam_interfaces__msg__MultiObjectOdometryPath__Sequence__create(size_t size);

/// Destroy array of msg/MultiObjectOdometryPath messages.
/**
 * It calls
 * dynamic_slam_interfaces__msg__MultiObjectOdometryPath__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_dynamic_slam_interfaces
void
dynamic_slam_interfaces__msg__MultiObjectOdometryPath__Sequence__destroy(dynamic_slam_interfaces__msg__MultiObjectOdometryPath__Sequence * array);

/// Check for msg/MultiObjectOdometryPath message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_dynamic_slam_interfaces
bool
dynamic_slam_interfaces__msg__MultiObjectOdometryPath__Sequence__are_equal(const dynamic_slam_interfaces__msg__MultiObjectOdometryPath__Sequence * lhs, const dynamic_slam_interfaces__msg__MultiObjectOdometryPath__Sequence * rhs);

/// Copy an array of msg/MultiObjectOdometryPath messages.
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
ROSIDL_GENERATOR_C_PUBLIC_dynamic_slam_interfaces
bool
dynamic_slam_interfaces__msg__MultiObjectOdometryPath__Sequence__copy(
  const dynamic_slam_interfaces__msg__MultiObjectOdometryPath__Sequence * input,
  dynamic_slam_interfaces__msg__MultiObjectOdometryPath__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // DYNAMIC_SLAM_INTERFACES__MSG__DETAIL__MULTI_OBJECT_ODOMETRY_PATH__FUNCTIONS_H_
