// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from dynamic_slam_interfaces:msg/ObjectOdometry.idl
// generated code does not contain a copyright notice
#include "dynamic_slam_interfaces/msg/detail/object_odometry__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `odom`
#include "nav_msgs/msg/detail/odometry__functions.h"
// Member `h_w_km1_k`
#include "geometry_msgs/msg/detail/pose_with_covariance__functions.h"

bool
dynamic_slam_interfaces__msg__ObjectOdometry__init(dynamic_slam_interfaces__msg__ObjectOdometry * msg)
{
  if (!msg) {
    return false;
  }
  // odom
  if (!nav_msgs__msg__Odometry__init(&msg->odom)) {
    dynamic_slam_interfaces__msg__ObjectOdometry__fini(msg);
    return false;
  }
  // h_w_km1_k
  if (!geometry_msgs__msg__PoseWithCovariance__init(&msg->h_w_km1_k)) {
    dynamic_slam_interfaces__msg__ObjectOdometry__fini(msg);
    return false;
  }
  // object_id
  // sequence
  return true;
}

void
dynamic_slam_interfaces__msg__ObjectOdometry__fini(dynamic_slam_interfaces__msg__ObjectOdometry * msg)
{
  if (!msg) {
    return;
  }
  // odom
  nav_msgs__msg__Odometry__fini(&msg->odom);
  // h_w_km1_k
  geometry_msgs__msg__PoseWithCovariance__fini(&msg->h_w_km1_k);
  // object_id
  // sequence
}

bool
dynamic_slam_interfaces__msg__ObjectOdometry__are_equal(const dynamic_slam_interfaces__msg__ObjectOdometry * lhs, const dynamic_slam_interfaces__msg__ObjectOdometry * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // odom
  if (!nav_msgs__msg__Odometry__are_equal(
      &(lhs->odom), &(rhs->odom)))
  {
    return false;
  }
  // h_w_km1_k
  if (!geometry_msgs__msg__PoseWithCovariance__are_equal(
      &(lhs->h_w_km1_k), &(rhs->h_w_km1_k)))
  {
    return false;
  }
  // object_id
  if (lhs->object_id != rhs->object_id) {
    return false;
  }
  // sequence
  if (lhs->sequence != rhs->sequence) {
    return false;
  }
  return true;
}

bool
dynamic_slam_interfaces__msg__ObjectOdometry__copy(
  const dynamic_slam_interfaces__msg__ObjectOdometry * input,
  dynamic_slam_interfaces__msg__ObjectOdometry * output)
{
  if (!input || !output) {
    return false;
  }
  // odom
  if (!nav_msgs__msg__Odometry__copy(
      &(input->odom), &(output->odom)))
  {
    return false;
  }
  // h_w_km1_k
  if (!geometry_msgs__msg__PoseWithCovariance__copy(
      &(input->h_w_km1_k), &(output->h_w_km1_k)))
  {
    return false;
  }
  // object_id
  output->object_id = input->object_id;
  // sequence
  output->sequence = input->sequence;
  return true;
}

dynamic_slam_interfaces__msg__ObjectOdometry *
dynamic_slam_interfaces__msg__ObjectOdometry__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dynamic_slam_interfaces__msg__ObjectOdometry * msg = (dynamic_slam_interfaces__msg__ObjectOdometry *)allocator.allocate(sizeof(dynamic_slam_interfaces__msg__ObjectOdometry), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(dynamic_slam_interfaces__msg__ObjectOdometry));
  bool success = dynamic_slam_interfaces__msg__ObjectOdometry__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
dynamic_slam_interfaces__msg__ObjectOdometry__destroy(dynamic_slam_interfaces__msg__ObjectOdometry * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    dynamic_slam_interfaces__msg__ObjectOdometry__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
dynamic_slam_interfaces__msg__ObjectOdometry__Sequence__init(dynamic_slam_interfaces__msg__ObjectOdometry__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dynamic_slam_interfaces__msg__ObjectOdometry * data = NULL;

  if (size) {
    data = (dynamic_slam_interfaces__msg__ObjectOdometry *)allocator.zero_allocate(size, sizeof(dynamic_slam_interfaces__msg__ObjectOdometry), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = dynamic_slam_interfaces__msg__ObjectOdometry__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        dynamic_slam_interfaces__msg__ObjectOdometry__fini(&data[i - 1]);
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
dynamic_slam_interfaces__msg__ObjectOdometry__Sequence__fini(dynamic_slam_interfaces__msg__ObjectOdometry__Sequence * array)
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
      dynamic_slam_interfaces__msg__ObjectOdometry__fini(&array->data[i]);
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

dynamic_slam_interfaces__msg__ObjectOdometry__Sequence *
dynamic_slam_interfaces__msg__ObjectOdometry__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dynamic_slam_interfaces__msg__ObjectOdometry__Sequence * array = (dynamic_slam_interfaces__msg__ObjectOdometry__Sequence *)allocator.allocate(sizeof(dynamic_slam_interfaces__msg__ObjectOdometry__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = dynamic_slam_interfaces__msg__ObjectOdometry__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
dynamic_slam_interfaces__msg__ObjectOdometry__Sequence__destroy(dynamic_slam_interfaces__msg__ObjectOdometry__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    dynamic_slam_interfaces__msg__ObjectOdometry__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
dynamic_slam_interfaces__msg__ObjectOdometry__Sequence__are_equal(const dynamic_slam_interfaces__msg__ObjectOdometry__Sequence * lhs, const dynamic_slam_interfaces__msg__ObjectOdometry__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!dynamic_slam_interfaces__msg__ObjectOdometry__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
dynamic_slam_interfaces__msg__ObjectOdometry__Sequence__copy(
  const dynamic_slam_interfaces__msg__ObjectOdometry__Sequence * input,
  dynamic_slam_interfaces__msg__ObjectOdometry__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(dynamic_slam_interfaces__msg__ObjectOdometry);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    dynamic_slam_interfaces__msg__ObjectOdometry * data =
      (dynamic_slam_interfaces__msg__ObjectOdometry *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!dynamic_slam_interfaces__msg__ObjectOdometry__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          dynamic_slam_interfaces__msg__ObjectOdometry__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!dynamic_slam_interfaces__msg__ObjectOdometry__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
