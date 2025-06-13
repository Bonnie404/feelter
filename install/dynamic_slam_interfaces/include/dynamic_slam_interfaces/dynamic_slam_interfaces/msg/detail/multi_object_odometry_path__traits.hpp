// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from dynamic_slam_interfaces:msg/MultiObjectOdometryPath.idl
// generated code does not contain a copyright notice

#ifndef DYNAMIC_SLAM_INTERFACES__MSG__DETAIL__MULTI_OBJECT_ODOMETRY_PATH__TRAITS_HPP_
#define DYNAMIC_SLAM_INTERFACES__MSG__DETAIL__MULTI_OBJECT_ODOMETRY_PATH__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "dynamic_slam_interfaces/msg/detail/multi_object_odometry_path__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'paths'
#include "dynamic_slam_interfaces/msg/detail/object_odometry_path__traits.hpp"

namespace dynamic_slam_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const MultiObjectOdometryPath & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: paths
  {
    if (msg.paths.size() == 0) {
      out << "paths: []";
    } else {
      out << "paths: [";
      size_t pending_items = msg.paths.size();
      for (auto item : msg.paths) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const MultiObjectOdometryPath & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: paths
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.paths.size() == 0) {
      out << "paths: []\n";
    } else {
      out << "paths:\n";
      for (auto item : msg.paths) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MultiObjectOdometryPath & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace dynamic_slam_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use dynamic_slam_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const dynamic_slam_interfaces::msg::MultiObjectOdometryPath & msg,
  std::ostream & out, size_t indentation = 0)
{
  dynamic_slam_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use dynamic_slam_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const dynamic_slam_interfaces::msg::MultiObjectOdometryPath & msg)
{
  return dynamic_slam_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<dynamic_slam_interfaces::msg::MultiObjectOdometryPath>()
{
  return "dynamic_slam_interfaces::msg::MultiObjectOdometryPath";
}

template<>
inline const char * name<dynamic_slam_interfaces::msg::MultiObjectOdometryPath>()
{
  return "dynamic_slam_interfaces/msg/MultiObjectOdometryPath";
}

template<>
struct has_fixed_size<dynamic_slam_interfaces::msg::MultiObjectOdometryPath>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<dynamic_slam_interfaces::msg::MultiObjectOdometryPath>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<dynamic_slam_interfaces::msg::MultiObjectOdometryPath>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // DYNAMIC_SLAM_INTERFACES__MSG__DETAIL__MULTI_OBJECT_ODOMETRY_PATH__TRAITS_HPP_
