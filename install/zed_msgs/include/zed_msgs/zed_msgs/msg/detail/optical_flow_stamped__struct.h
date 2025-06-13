// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from zed_msgs:msg/OpticalFlowStamped.idl
// generated code does not contain a copyright notice

#ifndef ZED_MSGS__MSG__DETAIL__OPTICAL_FLOW_STAMPED__STRUCT_H_
#define ZED_MSGS__MSG__DETAIL__OPTICAL_FLOW_STAMPED__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'flow_image'
#include "sensor_msgs/msg/detail/image__struct.h"

/// Struct defined in msg/OpticalFlowStamped in the package zed_msgs.
/**
  * Standard Header
 */
typedef struct zed_msgs__msg__OpticalFlowStamped
{
  std_msgs__msg__Header header;
  /// Optical Flow image
  /// The image represents the optical flow field.
  /// Each pixel corresponds to a motion vector (dx, dy).
  /// - Image encoding is '32FC2'
  /// - Each pixel contains two float32 values:
  ///   - channel 0: motion in x direction
  ///   - channel 1: motion in y direction
  sensor_msgs__msg__Image flow_image;
} zed_msgs__msg__OpticalFlowStamped;

// Struct for a sequence of zed_msgs__msg__OpticalFlowStamped.
typedef struct zed_msgs__msg__OpticalFlowStamped__Sequence
{
  zed_msgs__msg__OpticalFlowStamped * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} zed_msgs__msg__OpticalFlowStamped__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ZED_MSGS__MSG__DETAIL__OPTICAL_FLOW_STAMPED__STRUCT_H_
