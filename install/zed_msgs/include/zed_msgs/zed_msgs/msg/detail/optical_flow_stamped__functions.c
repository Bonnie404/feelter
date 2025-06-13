// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from zed_msgs:msg/OpticalFlowStamped.idl
// generated code does not contain a copyright notice
#include "zed_msgs/msg/detail/optical_flow_stamped__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `flow_image`
#include "sensor_msgs/msg/detail/image__functions.h"

bool
zed_msgs__msg__OpticalFlowStamped__init(zed_msgs__msg__OpticalFlowStamped * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    zed_msgs__msg__OpticalFlowStamped__fini(msg);
    return false;
  }
  // flow_image
  if (!sensor_msgs__msg__Image__init(&msg->flow_image)) {
    zed_msgs__msg__OpticalFlowStamped__fini(msg);
    return false;
  }
  return true;
}

void
zed_msgs__msg__OpticalFlowStamped__fini(zed_msgs__msg__OpticalFlowStamped * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // flow_image
  sensor_msgs__msg__Image__fini(&msg->flow_image);
}

bool
zed_msgs__msg__OpticalFlowStamped__are_equal(const zed_msgs__msg__OpticalFlowStamped * lhs, const zed_msgs__msg__OpticalFlowStamped * rhs)
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
  // flow_image
  if (!sensor_msgs__msg__Image__are_equal(
      &(lhs->flow_image), &(rhs->flow_image)))
  {
    return false;
  }
  return true;
}

bool
zed_msgs__msg__OpticalFlowStamped__copy(
  const zed_msgs__msg__OpticalFlowStamped * input,
  zed_msgs__msg__OpticalFlowStamped * output)
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
  // flow_image
  if (!sensor_msgs__msg__Image__copy(
      &(input->flow_image), &(output->flow_image)))
  {
    return false;
  }
  return true;
}

zed_msgs__msg__OpticalFlowStamped *
zed_msgs__msg__OpticalFlowStamped__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  zed_msgs__msg__OpticalFlowStamped * msg = (zed_msgs__msg__OpticalFlowStamped *)allocator.allocate(sizeof(zed_msgs__msg__OpticalFlowStamped), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(zed_msgs__msg__OpticalFlowStamped));
  bool success = zed_msgs__msg__OpticalFlowStamped__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
zed_msgs__msg__OpticalFlowStamped__destroy(zed_msgs__msg__OpticalFlowStamped * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    zed_msgs__msg__OpticalFlowStamped__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
zed_msgs__msg__OpticalFlowStamped__Sequence__init(zed_msgs__msg__OpticalFlowStamped__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  zed_msgs__msg__OpticalFlowStamped * data = NULL;

  if (size) {
    data = (zed_msgs__msg__OpticalFlowStamped *)allocator.zero_allocate(size, sizeof(zed_msgs__msg__OpticalFlowStamped), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = zed_msgs__msg__OpticalFlowStamped__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        zed_msgs__msg__OpticalFlowStamped__fini(&data[i - 1]);
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
zed_msgs__msg__OpticalFlowStamped__Sequence__fini(zed_msgs__msg__OpticalFlowStamped__Sequence * array)
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
      zed_msgs__msg__OpticalFlowStamped__fini(&array->data[i]);
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

zed_msgs__msg__OpticalFlowStamped__Sequence *
zed_msgs__msg__OpticalFlowStamped__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  zed_msgs__msg__OpticalFlowStamped__Sequence * array = (zed_msgs__msg__OpticalFlowStamped__Sequence *)allocator.allocate(sizeof(zed_msgs__msg__OpticalFlowStamped__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = zed_msgs__msg__OpticalFlowStamped__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
zed_msgs__msg__OpticalFlowStamped__Sequence__destroy(zed_msgs__msg__OpticalFlowStamped__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    zed_msgs__msg__OpticalFlowStamped__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
zed_msgs__msg__OpticalFlowStamped__Sequence__are_equal(const zed_msgs__msg__OpticalFlowStamped__Sequence * lhs, const zed_msgs__msg__OpticalFlowStamped__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!zed_msgs__msg__OpticalFlowStamped__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
zed_msgs__msg__OpticalFlowStamped__Sequence__copy(
  const zed_msgs__msg__OpticalFlowStamped__Sequence * input,
  zed_msgs__msg__OpticalFlowStamped__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(zed_msgs__msg__OpticalFlowStamped);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    zed_msgs__msg__OpticalFlowStamped * data =
      (zed_msgs__msg__OpticalFlowStamped *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!zed_msgs__msg__OpticalFlowStamped__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          zed_msgs__msg__OpticalFlowStamped__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!zed_msgs__msg__OpticalFlowStamped__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
