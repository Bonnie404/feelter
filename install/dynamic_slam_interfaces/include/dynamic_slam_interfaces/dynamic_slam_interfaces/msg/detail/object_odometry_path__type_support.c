// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from dynamic_slam_interfaces:msg/ObjectOdometryPath.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "dynamic_slam_interfaces/msg/detail/object_odometry_path__rosidl_typesupport_introspection_c.h"
#include "dynamic_slam_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "dynamic_slam_interfaces/msg/detail/object_odometry_path__functions.h"
#include "dynamic_slam_interfaces/msg/detail/object_odometry_path__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `colour`
#include "std_msgs/msg/color_rgba.h"
// Member `colour`
#include "std_msgs/msg/detail/color_rgba__rosidl_typesupport_introspection_c.h"
// Member `object_odometries`
#include "dynamic_slam_interfaces/msg/object_odometry.h"
// Member `object_odometries`
#include "dynamic_slam_interfaces/msg/detail/object_odometry__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void dynamic_slam_interfaces__msg__ObjectOdometryPath__rosidl_typesupport_introspection_c__ObjectOdometryPath_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  dynamic_slam_interfaces__msg__ObjectOdometryPath__init(message_memory);
}

void dynamic_slam_interfaces__msg__ObjectOdometryPath__rosidl_typesupport_introspection_c__ObjectOdometryPath_fini_function(void * message_memory)
{
  dynamic_slam_interfaces__msg__ObjectOdometryPath__fini(message_memory);
}

size_t dynamic_slam_interfaces__msg__ObjectOdometryPath__rosidl_typesupport_introspection_c__size_function__ObjectOdometryPath__object_odometries(
  const void * untyped_member)
{
  const dynamic_slam_interfaces__msg__ObjectOdometry__Sequence * member =
    (const dynamic_slam_interfaces__msg__ObjectOdometry__Sequence *)(untyped_member);
  return member->size;
}

const void * dynamic_slam_interfaces__msg__ObjectOdometryPath__rosidl_typesupport_introspection_c__get_const_function__ObjectOdometryPath__object_odometries(
  const void * untyped_member, size_t index)
{
  const dynamic_slam_interfaces__msg__ObjectOdometry__Sequence * member =
    (const dynamic_slam_interfaces__msg__ObjectOdometry__Sequence *)(untyped_member);
  return &member->data[index];
}

void * dynamic_slam_interfaces__msg__ObjectOdometryPath__rosidl_typesupport_introspection_c__get_function__ObjectOdometryPath__object_odometries(
  void * untyped_member, size_t index)
{
  dynamic_slam_interfaces__msg__ObjectOdometry__Sequence * member =
    (dynamic_slam_interfaces__msg__ObjectOdometry__Sequence *)(untyped_member);
  return &member->data[index];
}

void dynamic_slam_interfaces__msg__ObjectOdometryPath__rosidl_typesupport_introspection_c__fetch_function__ObjectOdometryPath__object_odometries(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const dynamic_slam_interfaces__msg__ObjectOdometry * item =
    ((const dynamic_slam_interfaces__msg__ObjectOdometry *)
    dynamic_slam_interfaces__msg__ObjectOdometryPath__rosidl_typesupport_introspection_c__get_const_function__ObjectOdometryPath__object_odometries(untyped_member, index));
  dynamic_slam_interfaces__msg__ObjectOdometry * value =
    (dynamic_slam_interfaces__msg__ObjectOdometry *)(untyped_value);
  *value = *item;
}

void dynamic_slam_interfaces__msg__ObjectOdometryPath__rosidl_typesupport_introspection_c__assign_function__ObjectOdometryPath__object_odometries(
  void * untyped_member, size_t index, const void * untyped_value)
{
  dynamic_slam_interfaces__msg__ObjectOdometry * item =
    ((dynamic_slam_interfaces__msg__ObjectOdometry *)
    dynamic_slam_interfaces__msg__ObjectOdometryPath__rosidl_typesupport_introspection_c__get_function__ObjectOdometryPath__object_odometries(untyped_member, index));
  const dynamic_slam_interfaces__msg__ObjectOdometry * value =
    (const dynamic_slam_interfaces__msg__ObjectOdometry *)(untyped_value);
  *item = *value;
}

bool dynamic_slam_interfaces__msg__ObjectOdometryPath__rosidl_typesupport_introspection_c__resize_function__ObjectOdometryPath__object_odometries(
  void * untyped_member, size_t size)
{
  dynamic_slam_interfaces__msg__ObjectOdometry__Sequence * member =
    (dynamic_slam_interfaces__msg__ObjectOdometry__Sequence *)(untyped_member);
  dynamic_slam_interfaces__msg__ObjectOdometry__Sequence__fini(member);
  return dynamic_slam_interfaces__msg__ObjectOdometry__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember dynamic_slam_interfaces__msg__ObjectOdometryPath__rosidl_typesupport_introspection_c__ObjectOdometryPath_message_member_array[5] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dynamic_slam_interfaces__msg__ObjectOdometryPath, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "colour",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dynamic_slam_interfaces__msg__ObjectOdometryPath, colour),  // bytes offset in struct
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
    offsetof(dynamic_slam_interfaces__msg__ObjectOdometryPath, object_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "path_segment",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT64,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dynamic_slam_interfaces__msg__ObjectOdometryPath, path_segment),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "object_odometries",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dynamic_slam_interfaces__msg__ObjectOdometryPath, object_odometries),  // bytes offset in struct
    NULL,  // default value
    dynamic_slam_interfaces__msg__ObjectOdometryPath__rosidl_typesupport_introspection_c__size_function__ObjectOdometryPath__object_odometries,  // size() function pointer
    dynamic_slam_interfaces__msg__ObjectOdometryPath__rosidl_typesupport_introspection_c__get_const_function__ObjectOdometryPath__object_odometries,  // get_const(index) function pointer
    dynamic_slam_interfaces__msg__ObjectOdometryPath__rosidl_typesupport_introspection_c__get_function__ObjectOdometryPath__object_odometries,  // get(index) function pointer
    dynamic_slam_interfaces__msg__ObjectOdometryPath__rosidl_typesupport_introspection_c__fetch_function__ObjectOdometryPath__object_odometries,  // fetch(index, &value) function pointer
    dynamic_slam_interfaces__msg__ObjectOdometryPath__rosidl_typesupport_introspection_c__assign_function__ObjectOdometryPath__object_odometries,  // assign(index, value) function pointer
    dynamic_slam_interfaces__msg__ObjectOdometryPath__rosidl_typesupport_introspection_c__resize_function__ObjectOdometryPath__object_odometries  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers dynamic_slam_interfaces__msg__ObjectOdometryPath__rosidl_typesupport_introspection_c__ObjectOdometryPath_message_members = {
  "dynamic_slam_interfaces__msg",  // message namespace
  "ObjectOdometryPath",  // message name
  5,  // number of fields
  sizeof(dynamic_slam_interfaces__msg__ObjectOdometryPath),
  dynamic_slam_interfaces__msg__ObjectOdometryPath__rosidl_typesupport_introspection_c__ObjectOdometryPath_message_member_array,  // message members
  dynamic_slam_interfaces__msg__ObjectOdometryPath__rosidl_typesupport_introspection_c__ObjectOdometryPath_init_function,  // function to initialize message memory (memory has to be allocated)
  dynamic_slam_interfaces__msg__ObjectOdometryPath__rosidl_typesupport_introspection_c__ObjectOdometryPath_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t dynamic_slam_interfaces__msg__ObjectOdometryPath__rosidl_typesupport_introspection_c__ObjectOdometryPath_message_type_support_handle = {
  0,
  &dynamic_slam_interfaces__msg__ObjectOdometryPath__rosidl_typesupport_introspection_c__ObjectOdometryPath_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_dynamic_slam_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, dynamic_slam_interfaces, msg, ObjectOdometryPath)() {
  dynamic_slam_interfaces__msg__ObjectOdometryPath__rosidl_typesupport_introspection_c__ObjectOdometryPath_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  dynamic_slam_interfaces__msg__ObjectOdometryPath__rosidl_typesupport_introspection_c__ObjectOdometryPath_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, ColorRGBA)();
  dynamic_slam_interfaces__msg__ObjectOdometryPath__rosidl_typesupport_introspection_c__ObjectOdometryPath_message_member_array[4].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, dynamic_slam_interfaces, msg, ObjectOdometry)();
  if (!dynamic_slam_interfaces__msg__ObjectOdometryPath__rosidl_typesupport_introspection_c__ObjectOdometryPath_message_type_support_handle.typesupport_identifier) {
    dynamic_slam_interfaces__msg__ObjectOdometryPath__rosidl_typesupport_introspection_c__ObjectOdometryPath_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &dynamic_slam_interfaces__msg__ObjectOdometryPath__rosidl_typesupport_introspection_c__ObjectOdometryPath_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
