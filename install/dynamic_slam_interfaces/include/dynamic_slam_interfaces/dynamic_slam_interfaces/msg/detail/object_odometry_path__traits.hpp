// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from dynamic_slam_interfaces:msg/ObjectOdometryPath.idl
// generated code does not contain a copyright notice

#ifndef DYNAMIC_SLAM_INTERFACES__MSG__DETAIL__OBJECT_ODOMETRY_PATH__TRAITS_HPP_
#define DYNAMIC_SLAM_INTERFACES__MSG__DETAIL__OBJECT_ODOMETRY_PATH__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "dynamic_slam_interfaces/msg/detail/object_odometry_path__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'colour'
#include "std_msgs/msg/detail/color_rgba__traits.hpp"
// Member 'object_odometries'
#include "dynamic_slam_interfaces/msg/detail/object_odometry__traits.hpp"

namespace dynamic_slam_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const ObjectOdometryPath & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: colour
  {
    out << "colour: ";
    to_flow_style_yaml(msg.colour, out);
    out << ", ";
  }

  // member: object_id
  {
    out << "object_id: ";
    rosidl_generator_traits::value_to_yaml(msg.object_id, out);
    out << ", ";
  }

  // member: path_segment
  {
    out << "path_segment: ";
    rosidl_generator_traits::value_to_yaml(msg.path_segment, out);
    out << ", ";
  }

  // member: object_odometries
  {
    if (msg.object_odometries.size() == 0) {
      out << "object_odometries: []";
    } else {
      out << "object_odometries: [";
      size_t pending_items = msg.object_odometries.size();
      for (auto item : msg.object_odometries) {
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
  const ObjectOdometryPath & msg,
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

  // member: colour
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "colour:\n";
    to_block_style_yaml(msg.colour, out, indentation + 2);
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

  // member: path_segment
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "path_segment: ";
    rosidl_generator_traits::value_to_yaml(msg.path_segment, out);
    out << "\n";
  }

  // member: object_odometries
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.object_odometries.size() == 0) {
      out << "object_odometries: []\n";
    } else {
      out << "object_odometries:\n";
      for (auto item : msg.object_odometries) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ObjectOdometryPath & msg, bool use_flow_style = false)
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
  const dynamic_slam_interfaces::msg::ObjectOdometryPath & msg,
  std::ostream & out, size_t indentation = 0)
{
  dynamic_slam_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use dynamic_slam_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const dynamic_slam_interfaces::msg::ObjectOdometryPath & msg)
{
  return dynamic_slam_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<dynamic_slam_interfaces::msg::ObjectOdometryPath>()
{
  return "dynamic_slam_interfaces::msg::ObjectOdometryPath";
}

template<>
inline const char * name<dynamic_slam_interfaces::msg::ObjectOdometryPath>()
{
  return "dynamic_slam_interfaces/msg/ObjectOdometryPath";
}

template<>
struct has_fixed_size<dynamic_slam_interfaces::msg::ObjectOdometryPath>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<dynamic_slam_interfaces::msg::ObjectOdometryPath>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<dynamic_slam_interfaces::msg::ObjectOdometryPath>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // DYNAMIC_SLAM_INTERFACES__MSG__DETAIL__OBJECT_ODOMETRY_PATH__TRAITS_HPP_
