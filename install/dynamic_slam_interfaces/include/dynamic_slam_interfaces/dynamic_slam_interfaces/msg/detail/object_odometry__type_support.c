// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from dynamic_slam_interfaces:msg/ObjectOdometry.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "dynamic_slam_interfaces/msg/detail/object_odometry__rosidl_typesupport_introspection_c.h"
#include "dynamic_slam_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "dynamic_slam_interfaces/msg/detail/object_odometry__functions.h"
#include "dynamic_slam_interfaces/msg/detail/object_odometry__struct.h"


// Include directives for member types
// Member `odom`
#include "nav_msgs/msg/odometry.h"
// Member `odom`
#include "nav_msgs/msg/detail/odometry__rosidl_typesupport_introspection_c.h"
// Member `h_w_km1_k`
#include "geometry_msgs/msg/pose_with_covariance.h"
// Member `h_w_km1_k`
#include "geometry_msgs/msg/detail/pose_with_covariance__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void dynamic_slam_interfaces__msg__ObjectOdometry__rosidl_typesupport_introspection_c__ObjectOdometry_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  dynamic_slam_interfaces__msg__ObjectOdometry__init(message_memory);
}

void dynamic_slam_interfaces__msg__ObjectOdometry__rosidl_typesupport_introspection_c__ObjectOdometry_fini_function(void * message_memory)
{
  dynamic_slam_interfaces__msg__ObjectOdometry__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember dynamic_slam_interfaces__msg__ObjectOdometry__rosidl_typesupport_introspection_c__ObjectOdometry_message_member_array[4] = {
  {
    "odom",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dynamic_slam_interfaces__msg__ObjectOdometry, odom),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "h_w_km1_k",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dynamic_slam_interfaces__msg__ObjectOdometry, h_w_km1_k),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "object_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT64,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dynamic_slam_interfaces__msg__ObjectOdometry, object_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "sequence",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dynamic_slam_interfaces__msg__ObjectOdometry, sequence),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers dynamic_slam_interfaces__msg__ObjectOdometry__rosidl_typesupport_introspection_c__ObjectOdometry_message_members = {
  "dynamic_slam_interfaces__msg",  // message namespace
  "ObjectOdometry",  // message name
  4,  // number of fields
  sizeof(dynamic_slam_interfaces__msg__ObjectOdometry),
  dynamic_slam_interfaces__msg__ObjectOdometry__rosidl_typesupport_introspection_c__ObjectOdometry_message_member_array,  // message members
  dynamic_slam_interfaces__msg__ObjectOdometry__rosidl_typesupport_introspection_c__ObjectOdometry_init_function,  // function to initialize message memory (memory has to be allocated)
  dynamic_slam_interfaces__msg__ObjectOdometry__rosidl_typesupport_introspection_c__ObjectOdometry_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t dynamic_slam_interfaces__msg__ObjectOdometry__rosidl_typesupport_introspection_c__ObjectOdometry_message_type_support_handle = {
  0,
  &dynamic_slam_interfaces__msg__ObjectOdometry__rosidl_typesupport_introspection_c__ObjectOdometry_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_dynamic_slam_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, dynamic_slam_interfaces, msg, ObjectOdometry)() {
  dynamic_slam_interfaces__msg__ObjectOdometry__rosidl_typesupport_introspection_c__ObjectOdometry_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, nav_msgs, msg, Odometry)();
  dynamic_slam_interfaces__msg__ObjectOdometry__rosidl_typesupport_introspection_c__ObjectOdometry_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, PoseWithCovariance)();
  if (!dynamic_slam_interfaces__msg__ObjectOdometry__rosidl_typesupport_introspection_c__ObjectOdometry_message_type_support_handle.typesupport_identifier) {
    dynamic_slam_interfaces__msg__ObjectOdometry__rosidl_typesupport_introspection_c__ObjectOdometry_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &dynamic_slam_interfaces__msg__ObjectOdometry__rosidl_typesupport_introspection_c__ObjectOdometry_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
