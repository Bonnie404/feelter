// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from dynamic_slam_interfaces:msg/MultiObjectOdometryPath.idl
// generated code does not contain a copyright notice

#ifndef DYNAMIC_SLAM_INTERFACES__MSG__DETAIL__MULTI_OBJECT_ODOMETRY_PATH__STRUCT_H_
#define DYNAMIC_SLAM_INTERFACES__MSG__DETAIL__MULTI_OBJECT_ODOMETRY_PATH__STRUCT_H_

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
// Member 'paths'
#include "dynamic_slam_interfaces/msg/detail/object_odometry_path__struct.h"

/// Struct defined in msg/MultiObjectOdometryPath in the package dynamic_slam_interfaces.
/**
  * Interface representing multiple object paths over time
  * header should be the current ROS time
 */
typedef struct dynamic_slam_interfaces__msg__MultiObjectOdometryPath
{
  std_msgs__msg__Header header;
  /// List of object odometry paths, one per object path segment
  dynamic_slam_interfaces__msg__ObjectOdometryPath__Sequence paths;
} dynamic_slam_interfaces__msg__MultiObjectOdometryPath;

// Struct for a sequence of dynamic_slam_interfaces__msg__MultiObjectOdometryPath.
typedef struct dynamic_slam_interfaces__msg__MultiObjectOdometryPath__Sequence
{
  dynamic_slam_interfaces__msg__MultiObjectOdometryPath * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} dynamic_slam_interfaces__msg__MultiObjectOdometryPath__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // DYNAMIC_SLAM_INTERFACES__MSG__DETAIL__MULTI_OBJECT_ODOMETRY_PATH__STRUCT_H_
