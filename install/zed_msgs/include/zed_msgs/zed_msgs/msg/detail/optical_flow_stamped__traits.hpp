// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from zed_msgs:msg/OpticalFlowStamped.idl
// generated code does not contain a copyright notice

#ifndef ZED_MSGS__MSG__DETAIL__OPTICAL_FLOW_STAMPED__TRAITS_HPP_
#define ZED_MSGS__MSG__DETAIL__OPTICAL_FLOW_STAMPED__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "zed_msgs/msg/detail/optical_flow_stamped__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'flow_image'
#include "sensor_msgs/msg/detail/image__traits.hpp"

namespace zed_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const OpticalFlowStamped & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: flow_image
  {
    out << "flow_image: ";
    to_flow_style_yaml(msg.flow_image, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const OpticalFlowStamped & msg,
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

  // member: flow_image
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "flow_image:\n";
    to_block_style_yaml(msg.flow_image, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const OpticalFlowStamped & msg, bool use_flow_style = false)
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

}  // namespace zed_msgs

namespace rosidl_generator_traits
{

[[deprecated("use zed_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const zed_msgs::msg::OpticalFlowStamped & msg,
  std::ostream & out, size_t indentation = 0)
{
  zed_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use zed_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const zed_msgs::msg::OpticalFlowStamped & msg)
{
  return zed_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<zed_msgs::msg::OpticalFlowStamped>()
{
  return "zed_msgs::msg::OpticalFlowStamped";
}

template<>
inline const char * name<zed_msgs::msg::OpticalFlowStamped>()
{
  return "zed_msgs/msg/OpticalFlowStamped";
}

template<>
struct has_fixed_size<zed_msgs::msg::OpticalFlowStamped>
  : std::integral_constant<bool, has_fixed_size<sensor_msgs::msg::Image>::value && has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<zed_msgs::msg::OpticalFlowStamped>
  : std::integral_constant<bool, has_bounded_size<sensor_msgs::msg::Image>::value && has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<zed_msgs::msg::OpticalFlowStamped>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ZED_MSGS__MSG__DETAIL__OPTICAL_FLOW_STAMPED__TRAITS_HPP_
