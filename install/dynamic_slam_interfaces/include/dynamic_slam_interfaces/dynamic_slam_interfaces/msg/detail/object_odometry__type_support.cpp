// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from dynamic_slam_interfaces:msg/ObjectOdometry.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "dynamic_slam_interfaces/msg/detail/object_odometry__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace dynamic_slam_interfaces
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void ObjectOdometry_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) dynamic_slam_interfaces::msg::ObjectOdometry(_init);
}

void ObjectOdometry_fini_function(void * message_memory)
{
  auto typed_message = static_cast<dynamic_slam_interfaces::msg::ObjectOdometry *>(message_memory);
  typed_message->~ObjectOdometry();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember ObjectOdometry_message_member_array[4] = {
  {
    "odom",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<nav_msgs::msg::Odometry>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dynamic_slam_interfaces::msg::ObjectOdometry, odom),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "h_w_km1_k",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<geometry_msgs::msg::PoseWithCovariance>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dynamic_slam_interfaces::msg::ObjectOdometry, h_w_km1_k),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "object_id",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT64,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dynamic_slam_interfaces::msg::ObjectOdometry, object_id),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "sequence",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dynamic_slam_interfaces::msg::ObjectOdometry, sequence),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers ObjectOdometry_message_members = {
  "dynamic_slam_interfaces::msg",  // message namespace
  "ObjectOdometry",  // message name
  4,  // number of fields
  sizeof(dynamic_slam_interfaces::msg::ObjectOdometry),
  ObjectOdometry_message_member_array,  // message members
  ObjectOdometry_init_function,  // function to initialize message memory (memory has to be allocated)
  ObjectOdometry_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t ObjectOdometry_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &ObjectOdometry_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace dynamic_slam_interfaces


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<dynamic_slam_interfaces::msg::ObjectOdometry>()
{
  return &::dynamic_slam_interfaces::msg::rosidl_typesupport_introspection_cpp::ObjectOdometry_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, dynamic_slam_interfaces, msg, ObjectOdometry)() {
  return &::dynamic_slam_interfaces::msg::rosidl_typesupport_introspection_cpp::ObjectOdometry_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
