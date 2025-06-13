// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from dynamic_slam_interfaces:msg/MultiObjectOdometryPath.idl
// generated code does not contain a copyright notice
#include "dynamic_slam_interfaces/msg/detail/multi_object_odometry_path__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `paths`
#include "dynamic_slam_interfaces/msg/detail/object_odometry_path__functions.h"

bool
dynamic_slam_interfaces__msg__MultiObjectOdometryPath__init(dynamic_slam_interfaces__msg__MultiObjectOdometryPath * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    dynamic_slam_interfaces__msg__MultiObjectOdometryPath__fini(msg);
    return false;
  }
  // paths
  if (!dynamic_slam_interfaces__msg__ObjectOdometryPath__Sequence__init(&msg->paths, 0)) {
    dynamic_slam_interfaces__msg__MultiObjectOdometryPath__fini(msg);
    return false;
  }
  return true;
}

void
dynamic_slam_interfaces__msg__MultiObjectOdometryPath__fini(dynamic_slam_interfaces__msg__MultiObjectOdometryPath * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // paths
  dynamic_slam_interfaces__msg__ObjectOdometryPath__Sequence__fini(&msg->paths);
}

bool
dynamic_slam_interfaces__msg__MultiObjectOdometryPath__are_equal(const dynamic_slam_interfaces__msg__MultiObjectOdometryPath * lhs, const dynamic_slam_interfaces__msg__MultiObjectOdometryPath * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // paths
  if (!dynamic_slam_interfaces__msg__ObjectOdometryPath__Sequence__are_equal(
      &(lhs->paths), &(rhs->paths)))
  {
    return false;
  }
  return true;
}

bool
dynamic_slam_interfaces__msg__MultiObjectOdometryPath__copy(
  const dynamic_slam_interfaces__msg__MultiObjectOdometryPath * input,
  dynamic_slam_interfaces__msg__MultiObjectOdometryPath * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // paths
  if (!dynamic_slam_interfaces__msg__ObjectOdometryPath__Sequence__copy(
      &(input->paths), &(output->paths)))
  {
    return false;
  }
  return true;
}

dynamic_slam_interfaces__msg__MultiObjectOdometryPath *
dynamic_slam_interfaces__msg__MultiObjectOdometryPath__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dynamic_slam_interfaces__msg__MultiObjectOdometryPath * msg = (dynamic_slam_interfaces__msg__MultiObjectOdometryPath *)allocator.allocate(sizeof(dynamic_slam_interfaces__msg__MultiObjectOdometryPath), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(dynamic_slam_interfaces__msg__MultiObjectOdometryPath));
  bool success = dynamic_slam_interfaces__msg__MultiObjectOdometryPath__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
dynamic_slam_interfaces__msg__MultiObjectOdometryPath__destroy(dynamic_slam_interfaces__msg__MultiObjectOdometryPath * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    dynamic_slam_interfaces__msg__MultiObjectOdometryPath__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
dynamic_slam_interfaces__msg__MultiObjectOdometryPath__Sequence__init(dynamic_slam_interfaces__msg__MultiObjectOdometryPath__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dynamic_slam_interfaces__msg__MultiObjectOdometryPath * data = NULL;

  if (size) {
    data = (dynamic_slam_interfaces__msg__MultiObjectOdometryPath *)allocator.zero_allocate(size, sizeof(dynamic_slam_interfaces__msg__MultiObjectOdometryPath), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = dynamic_slam_interfaces__msg__MultiObjectOdometryPath__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        dynamic_slam_interfaces__msg__MultiObjectOdometryPath__fini(&data[i - 1]);
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
dynamic_slam_interfaces__msg__MultiObjectOdometryPath__Sequence__fini(dynamic_slam_interfaces__msg__MultiObjectOdometryPath__Sequence * array)
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
      dynamic_slam_interfaces__msg__MultiObjectOdometryPath__fini(&array->data[i]);
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

dynamic_slam_interfaces__msg__MultiObjectOdometryPath__Sequence *
dynamic_slam_interfaces__msg__MultiObjectOdometryPath__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dynamic_slam_interfaces__msg__MultiObjectOdometryPath__Sequence * array = (dynamic_slam_interfaces__msg__MultiObjectOdometryPath__Sequence *)allocator.allocate(sizeof(dynamic_slam_interfaces__msg__MultiObjectOdometryPath__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = dynamic_slam_interfaces__msg__MultiObjectOdometryPath__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
dynamic_slam_interfaces__msg__MultiObjectOdometryPath__Sequence__destroy(dynamic_slam_interfaces__msg__MultiObjectOdometryPath__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    dynamic_slam_interfaces__msg__MultiObjectOdometryPath__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
dynamic_slam_interfaces__msg__MultiObjectOdometryPath__Sequence__are_equal(const dynamic_slam_interfaces__msg__MultiObjectOdometryPath__Sequence * lhs, const dynamic_slam_interfaces__msg__MultiObjectOdometryPath__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!dynamic_slam_interfaces__msg__MultiObjectOdometryPath__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
dynamic_slam_interfaces__msg__MultiObjectOdometryPath__Sequence__copy(
  const dynamic_slam_interfaces__msg__MultiObjectOdometryPath__Sequence * input,
  dynamic_slam_interfaces__msg__MultiObjectOdometryPath__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(dynamic_slam_interfaces__msg__MultiObjectOdometryPath);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    dynamic_slam_interfaces__msg__MultiObjectOdometryPath * data =
      (dynamic_slam_interfaces__msg__MultiObjectOdometryPath *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!dynamic_slam_interfaces__msg__MultiObjectOdometryPath__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          dynamic_slam_interfaces__msg__MultiObjectOdometryPath__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!dynamic_slam_interfaces__msg__MultiObjectOdometryPath__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
