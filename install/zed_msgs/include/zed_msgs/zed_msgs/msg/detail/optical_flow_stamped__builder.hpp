// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from zed_msgs:msg/OpticalFlowStamped.idl
// generated code does not contain a copyright notice

#ifndef ZED_MSGS__MSG__DETAIL__OPTICAL_FLOW_STAMPED__BUILDER_HPP_
#define ZED_MSGS__MSG__DETAIL__OPTICAL_FLOW_STAMPED__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "zed_msgs/msg/detail/optical_flow_stamped__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace zed_msgs
{

namespace msg
{

namespace builder
{

class Init_OpticalFlowStamped_flow_image
{
public:
  explicit Init_OpticalFlowStamped_flow_image(::zed_msgs::msg::OpticalFlowStamped & msg)
  : msg_(msg)
  {}
  ::zed_msgs::msg::OpticalFlowStamped flow_image(::zed_msgs::msg::OpticalFlowStamped::_flow_image_type arg)
  {
    msg_.flow_image = std::move(arg);
    return std::move(msg_);
  }

private:
  ::zed_msgs::msg::OpticalFlowStamped msg_;
};

class Init_OpticalFlowStamped_header
{
public:
  Init_OpticalFlowStamped_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_OpticalFlowStamped_flow_image header(::zed_msgs::msg::OpticalFlowStamped::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_OpticalFlowStamped_flow_image(msg_);
  }

private:
  ::zed_msgs::msg::OpticalFlowStamped msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::zed_msgs::msg::OpticalFlowStamped>()
{
  return zed_msgs::msg::builder::Init_OpticalFlowStamped_header();
}

}  // namespace zed_msgs

#endif  // ZED_MSGS__MSG__DETAIL__OPTICAL_FLOW_STAMPED__BUILDER_HPP_
