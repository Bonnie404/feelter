// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from dynamic_slam_interfaces:msg/MultiObjectOdometryPath.idl
// generated code does not contain a copyright notice

#ifndef DYNAMIC_SLAM_INTERFACES__MSG__DETAIL__MULTI_OBJECT_ODOMETRY_PATH__BUILDER_HPP_
#define DYNAMIC_SLAM_INTERFACES__MSG__DETAIL__MULTI_OBJECT_ODOMETRY_PATH__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "dynamic_slam_interfaces/msg/detail/multi_object_odometry_path__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace dynamic_slam_interfaces
{

namespace msg
{

namespace builder
{

class Init_MultiObjectOdometryPath_paths
{
public:
  explicit Init_MultiObjectOdometryPath_paths(::dynamic_slam_interfaces::msg::MultiObjectOdometryPath & msg)
  : msg_(msg)
  {}
  ::dynamic_slam_interfaces::msg::MultiObjectOdometryPath paths(::dynamic_slam_interfaces::msg::MultiObjectOdometryPath::_paths_type arg)
  {
    msg_.paths = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dynamic_slam_interfaces::msg::MultiObjectOdometryPath msg_;
};

class Init_MultiObjectOdometryPath_header
{
public:
  Init_MultiObjectOdometryPath_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MultiObjectOdometryPath_paths header(::dynamic_slam_interfaces::msg::MultiObjectOdometryPath::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_MultiObjectOdometryPath_paths(msg_);
  }

private:
  ::dynamic_slam_interfaces::msg::MultiObjectOdometryPath msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::dynamic_slam_interfaces::msg::MultiObjectOdometryPath>()
{
  return dynamic_slam_interfaces::msg::builder::Init_MultiObjectOdometryPath_header();
}

}  // namespace dynamic_slam_interfaces

#endif  // DYNAMIC_SLAM_INTERFACES__MSG__DETAIL__MULTI_OBJECT_ODOMETRY_PATH__BUILDER_HPP_
