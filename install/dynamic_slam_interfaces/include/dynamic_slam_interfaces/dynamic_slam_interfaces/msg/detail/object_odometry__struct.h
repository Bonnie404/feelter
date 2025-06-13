// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from dynamic_slam_interfaces:msg/ObjectOdometry.idl
// generated code does not contain a copyright notice

#ifndef DYNAMIC_SLAM_INTERFACES__MSG__DETAIL__OBJECT_ODOMETRY__STRUCT_H_
#define DYNAMIC_SLAM_INTERFACES__MSG__DETAIL__OBJECT_ODOMETRY__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'odom'
#include "nav_msgs/msg/detail/odometry__struct.h"
// Member 'h_w_km1_k'
#include "geometry_msgs/msg/detail/pose_with_covariance__struct.h"

/// Struct defined in msg/ObjectOdometry in the package dynamic_slam_interfaces.
/**
  * This represents the odometry (pose and velocity) of a single object with given id
  * We use the header from odom.header which should be the timestamp 
  * of the estimated motion/pose
 */
typedef struct dynamic_slam_interfaces__msg__ObjectOdometry
{
  /// object pose (L_w_k) and body-centric velocity.
  nav_msgs__msg__Odometry odom;
  /// motion in world from k-1 to k
  geometry_msgs__msg__PoseWithCovariance h_w_km1_k;
  /// unique object id (j)
  int64_t object_id;
  /// sequence ID: consecutively increasing ID (k)
  /// Sequence id's can be used to indicate when odometries are missing
  /// as they will be out of order (but should always be increasing)
  uint32_t sequence;
} dynamic_slam_interfaces__msg__ObjectOdometry;

// Struct for a sequence of dynamic_slam_interfaces__msg__ObjectOdometry.
typedef struct dynamic_slam_interfaces__msg__ObjectOdometry__Sequence
{
  dynamic_slam_interfaces__msg__ObjectOdometry * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} dynamic_slam_interfaces__msg__ObjectOdometry__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // DYNAMIC_SLAM_INTERFACES__MSG__DETAIL__OBJECT_ODOMETRY__STRUCT_H_
