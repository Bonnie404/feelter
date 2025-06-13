// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from dynamic_slam_interfaces:msg/ObjectOdometryPath.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "dynamic_slam_interfaces/msg/detail/object_odometry_path__struct.hpp"
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

void ObjectOdometryPath_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) dynamic_slam_interfaces::msg::ObjectOdometryPath(_init);
}

void ObjectOdometryPath_fini_function(void * message_memory)
{
  auto typed_message = static_cast<dynamic_slam_interfaces::msg::ObjectOdometryPath *>(message_memory);
  typed_message->~ObjectOdometryPath();
}

size_t size_function__ObjectOdometryPath__object_odometries(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<dynamic_slam_interfaces::msg::ObjectOdometry> *>(untyped_member);
  return member->size();
}

const void * get_const_function__ObjectOdometryPath__object_odometries(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<dynamic_slam_interfaces::msg::ObjectOdometry> *>(untyped_member);
  return &member[index];
}

void * get_function__ObjectOdometryPath__object_odometries(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<dynamic_slam_interfaces::msg::ObjectOdometry> *>(untyped_member);
  return &member[index];
}

void fetch_function__ObjectOdometryPath__object_odometries(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const dynamic_slam_interfaces::msg::ObjectOdometry *>(
    get_const_function__ObjectOdometryPath__object_odometries(untyped_member, index));
  auto & value = *reinterpret_cast<dynamic_slam_interfaces::msg::ObjectOdometry *>(untyped_value);
  value = item;
}

void assign_function__ObjectOdometryPath__object_odometries(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<dynamic_slam_interfaces::msg::ObjectOdometry *>(
    get_function__ObjectOdometryPath__object_odometries(untyped_member, index));
  const auto & value = *reinterpret_cast<const dynamic_slam_interfaces::msg::ObjectOdometry *>(untyped_value);
  item = value;
}

void resize_function__ObjectOdometryPath__object_odometries(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<dynamic_slam_interfaces::msg::ObjectOdometry> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember ObjectOdometryPath_message_member_array[5] = {
  {
    "header",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<std_msgs::msg::Header>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dynamic_slam_interfaces::msg::ObjectOdometryPath, header),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "colour",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<std_msgs::msg::ColorRGBA>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dynamic_slam_interfaces::msg::ObjectOdometryPath, colour),  // bytes offset in struct
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
    offsetof(dynamic_slam_interfaces::msg::ObjectOdometryPath, object_id),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "path_segment",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT64,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dynamic_slam_interfaces::msg::ObjectOdometryPath, path_segment),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "object_odometries",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<dynamic_slam_interfaces::msg::ObjectOdometry>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dynamic_slam_interfaces::msg::ObjectOdometryPath, object_odometries),  // bytes offset in struct
    nullptr,  // default value
    size_function__ObjectOdometryPath__object_odometries,  // size() function pointer
    get_const_function__ObjectOdometryPath__object_odometries,  // get_const(index) function pointer
    get_function__ObjectOdometryPath__object_odometries,  // get(index) function pointer
    fetch_function__ObjectOdometryPath__object_odometries,  // fetch(index, &value) function pointer
    assign_function__ObjectOdometryPath__object_odometries,  // assign(index, value) function pointer
    resize_function__ObjectOdometryPath__object_odometries  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers ObjectOdometryPath_message_members = {
  "dynamic_slam_interfaces::msg",  // message namespace
  "ObjectOdometryPath",  // message name
  5,  // number of fields
  sizeof(dynamic_slam_interfaces::msg::ObjectOdometryPath),
  ObjectOdometryPath_message_member_array,  // message members
  ObjectOdometryPath_init_function,  // function to initialize message memory (memory has to be allocated)
  ObjectOdometryPath_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t ObjectOdometryPath_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &ObjectOdometryPath_message_members,
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
get_message_type_support_handle<dynamic_slam_interfaces::msg::ObjectOdometryPath>()
{
  return &::dynamic_slam_interfaces::msg::rosidl_typesupport_introspection_cpp::ObjectOdometryPath_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, dynamic_slam_interfaces, msg, ObjectOdometryPath)() {
  return &::dynamic_slam_interfaces::msg::rosidl_typesupport_introspection_cpp::ObjectOdometryPath_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
