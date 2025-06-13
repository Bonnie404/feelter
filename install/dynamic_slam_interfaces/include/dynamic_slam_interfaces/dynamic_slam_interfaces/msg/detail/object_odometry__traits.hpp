// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from dynamic_slam_interfaces:msg/ObjectOdometry.idl
// generated code does not contain a copyright notice

#ifndef DYNAMIC_SLAM_INTERFACES__MSG__DETAIL__OBJECT_ODOMETRY__TRAITS_HPP_
#define DYNAMIC_SLAM_INTERFACES__MSG__DETAIL__OBJECT_ODOMETRY__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "dynamic_slam_interfaces/msg/detail/object_odometry__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'odom'
#include "nav_msgs/msg/detail/odometry__traits.hpp"
// Member 'h_w_km1_k'
#include "geometry_msgs/msg/detail/pose_with_covariance__traits.hpp"

namespace dynamic_slam_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const ObjectOdometry & msg,
  std::ostream & out)
{
  out << "{";
  // member: odom
  {
    out << "odom: ";
    to_flow_style_yaml(msg.odom, out);
    out << ", ";
  }

  // member: h_w_km1_k
  {
    out << "h_w_km1_k: ";
    to_flow_style_yaml(msg.h_w_km1_k, out);
    out << ", ";
  }

  // member: object_id
  {
    out << "object_id: ";
    rosidl_generator_traits::value_to_yaml(msg.object_id, out);
    out << ", ";
  }

  // member: sequence
  {
    out << "sequence: ";
    rosidl_generator_traits::value_to_yaml(msg.sequence, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ObjectOdometry & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: odom
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "odom:\n";
    to_block_style_yaml(msg.odom, out, indentation + 2);
  }

  // member: h_w_km1_k
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "h_w_km1_k:\n";
    to_block_style_yaml(msg.h_w_km1_k, out, indentation + 2);
  }

  // member: object_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "object_id: ";
    rosidl_generator_traits::value_to_yaml(msg.object_id, out);
    out << "\n";
  }

  // member: sequence
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "sequence: ";
    rosidl_generator_traits::value_to_yaml(msg.sequence, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ObjectOdometry & msg, bool use_flow_style = false)
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
  const dynamic_slam_interfaces::msg::ObjectOdometry & msg,
  std::ostream & out, size_t indentation = 0)
{
  dynamic_slam_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use dynamic_slam_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const dynamic_slam_interfaces::msg::ObjectOdometry & msg)
{
  return dynamic_slam_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<dynamic_slam_interfaces::msg::ObjectOdometry>()
{
  return "dynamic_slam_interfaces::msg::ObjectOdometry";
}

template<>
inline const char * name<dynamic_slam_interfaces::msg::ObjectOdometry>()
{
  return "dynamic_slam_interfaces/msg/ObjectOdometry";
}

template<>
struct has_fixed_size<dynamic_slam_interfaces::msg::ObjectOdometry>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::PoseWithCovariance>::value && has_fixed_size<nav_msgs::msg::Odometry>::value> {};

template<>
struct has_bounded_size<dynamic_slam_interfaces::msg::ObjectOdometry>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::PoseWithCovariance>::value && has_bounded_size<nav_msgs::msg::Odometry>::value> {};

template<>
struct is_message<dynamic_slam_interfaces::msg::ObjectOdometry>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // DYNAMIC_SLAM_INTERFACES__MSG__DETAIL__OBJECT_ODOMETRY__TRAITS_HPP_
