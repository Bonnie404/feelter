// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from dynamic_slam_interfaces:msg/ObjectOdometryPath.idl
// generated code does not contain a copyright notice

#ifndef DYNAMIC_SLAM_INTERFACES__MSG__DETAIL__OBJECT_ODOMETRY_PATH__STRUCT_H_
#define DYNAMIC_SLAM_INTERFACES__MSG__DETAIL__OBJECT_ODOMETRY_PATH__STRUCT_H_

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
// Member 'colour'
#include "std_msgs/msg/detail/color_rgba__struct.h"
// Member 'object_odometries'
#include "dynamic_slam_interfaces/msg/detail/object_odometry__struct.h"

/// Struct defined in msg/ObjectOdometryPath in the package dynamic_slam_interfaces.
typedef struct dynamic_slam_interfaces__msg__ObjectOdometryPath
{
  std_msgs__msg__Header header;
  std_msgs__msg__ColorRGBA colour;
  int64_t object_id;
  /// indicating which segment of the total object trajectory this message
  /// corresponds to. A single object(id) may have multiple segments
  int64_t path_segment;
  dynamic_slam_interfaces__msg__ObjectOdometry__Sequence object_odometries;
} dynamic_slam_interfaces__msg__ObjectOdometryPath;

// Struct for a sequence of dynamic_slam_interfaces__msg__ObjectOdometryPath.
typedef struct dynamic_slam_interfaces__msg__ObjectOdometryPath__Sequence
{
  dynamic_slam_interfaces__msg__ObjectOdometryPath * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} dynamic_slam_interfaces__msg__ObjectOdometryPath__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // DYNAMIC_SLAM_INTERFACES__MSG__DETAIL__OBJECT_ODOMETRY_PATH__STRUCT_H_
