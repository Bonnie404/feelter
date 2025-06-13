// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from dynamic_slam_interfaces:msg/MultiObjectOdometryPath.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "dynamic_slam_interfaces/msg/detail/multi_object_odometry_path__struct.hpp"
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

void MultiObjectOdometryPath_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) dynamic_slam_interfaces::msg::MultiObjectOdometryPath(_init);
}

void MultiObjectOdometryPath_fini_function(void * message_memory)
{
  auto typed_message = static_cast<dynamic_slam_interfaces::msg::MultiObjectOdometryPath *>(message_memory);
  typed_message->~MultiObjectOdometryPath();
}

size_t size_function__MultiObjectOdometryPath__paths(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<dynamic_slam_interfaces::msg::ObjectOdometryPath> *>(untyped_member);
  return member->size();
}

const void * get_const_function__MultiObjectOdometryPath__paths(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<dynamic_slam_interfaces::msg::ObjectOdometryPath> *>(untyped_member);
  return &member[index];
}

void * get_function__MultiObjectOdometryPath__paths(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<dynamic_slam_interfaces::msg::ObjectOdometryPath> *>(untyped_member);
  return &member[index];
}

void fetch_function__MultiObjectOdometryPath__paths(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const dynamic_slam_interfaces::msg::ObjectOdometryPath *>(
    get_const_function__MultiObjectOdometryPath__paths(untyped_member, index));
  auto & value = *reinterpret_cast<dynamic_slam_interfaces::msg::ObjectOdometryPath *>(untyped_value);
  value = item;
}

void assign_function__MultiObjectOdometryPath__paths(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<dynamic_slam_interfaces::msg::ObjectOdometryPath *>(
    get_function__MultiObjectOdometryPath__paths(untyped_member, index));
  const auto & value = *reinterpret_cast<const dynamic_slam_interfaces::msg::ObjectOdometryPath *>(untyped_value);
  item = value;
}

void resize_function__MultiObjectOdometryPath__paths(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<dynamic_slam_interfaces::msg::ObjectOdometryPath> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember MultiObjectOdometryPath_message_member_array[2] = {
  {
    "header",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<std_msgs::msg::Header>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dynamic_slam_interfaces::msg::MultiObjectOdometryPath, header),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "paths",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<dynamic_slam_interfaces::msg::ObjectOdometryPath>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dynamic_slam_interfaces::msg::MultiObjectOdometryPath, paths),  // bytes offset in struct
    nullptr,  // default value
    size_function__MultiObjectOdometryPath__paths,  // size() function pointer
    get_const_function__MultiObjectOdometryPath__paths,  // get_const(index) function pointer
    get_function__MultiObjectOdometryPath__paths,  // get(index) function pointer
    fetch_function__MultiObjectOdometryPath__paths,  // fetch(index, &value) function pointer
    assign_function__MultiObjectOdometryPath__paths,  // assign(index, value) function pointer
    resize_function__MultiObjectOdometryPath__paths  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers MultiObjectOdometryPath_message_members = {
  "dynamic_slam_interfaces::msg",  // message namespace
  "MultiObjectOdometryPath",  // message name
  2,  // number of fields
  sizeof(dynamic_slam_interfaces::msg::MultiObjectOdometryPath),
  MultiObjectOdometryPath_message_member_array,  // message members
  MultiObjectOdometryPath_init_function,  // function to initialize message memory (memory has to be allocated)
  MultiObjectOdometryPath_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t MultiObjectOdometryPath_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &MultiObjectOdometryPath_message_members,
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
get_message_type_support_handle<dynamic_slam_interfaces::msg::MultiObjectOdometryPath>()
{
  return &::dynamic_slam_interfaces::msg::rosidl_typesupport_introspection_cpp::MultiObjectOdometryPath_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, dynamic_slam_interfaces, msg, MultiObjectOdometryPath)() {
  return &::dynamic_slam_interfaces::msg::rosidl_typesupport_introspection_cpp::MultiObjectOdometryPath_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
