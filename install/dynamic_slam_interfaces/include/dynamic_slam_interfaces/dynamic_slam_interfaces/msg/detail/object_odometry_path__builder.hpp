// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from dynamic_slam_interfaces:msg/ObjectOdometryPath.idl
// generated code does not contain a copyright notice

#ifndef DYNAMIC_SLAM_INTERFACES__MSG__DETAIL__OBJECT_ODOMETRY_PATH__BUILDER_HPP_
#define DYNAMIC_SLAM_INTERFACES__MSG__DETAIL__OBJECT_ODOMETRY_PATH__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "dynamic_slam_interfaces/msg/detail/object_odometry_path__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace dynamic_slam_interfaces
{

namespace msg
{

namespace builder
{

class Init_ObjectOdometryPath_object_odometries
{
public:
  explicit Init_ObjectOdometryPath_object_odometries(::dynamic_slam_interfaces::msg::ObjectOdometryPath & msg)
  : msg_(msg)
  {}
  ::dynamic_slam_interfaces::msg::ObjectOdometryPath object_odometries(::dynamic_slam_interfaces::msg::ObjectOdometryPath::_object_odometries_type arg)
  {
    msg_.object_odometries = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dynamic_slam_interfaces::msg::ObjectOdometryPath msg_;
};

class Init_ObjectOdometryPath_path_segment
{
public:
  explicit Init_ObjectOdometryPath_path_segment(::dynamic_slam_interfaces::msg::ObjectOdometryPath & msg)
  : msg_(msg)
  {}
  Init_ObjectOdometryPath_object_odometries path_segment(::dynamic_slam_interfaces::msg::ObjectOdometryPath::_path_segment_type arg)
  {
    msg_.path_segment = std::move(arg);
    return Init_ObjectOdometryPath_object_odometries(msg_);
  }

private:
  ::dynamic_slam_interfaces::msg::ObjectOdometryPath msg_;
};

class Init_ObjectOdometryPath_object_id
{
public:
  explicit Init_ObjectOdometryPath_object_id(::dynamic_slam_interfaces::msg::ObjectOdometryPath & msg)
  : msg_(msg)
  {}
  Init_ObjectOdometryPath_path_segment object_id(::dynamic_slam_interfaces::msg::ObjectOdometryPath::_object_id_type arg)
  {
    msg_.object_id = std::move(arg);
    return Init_ObjectOdometryPath_path_segment(msg_);
  }

private:
  ::dynamic_slam_interfaces::msg::ObjectOdometryPath msg_;
};

class Init_ObjectOdometryPath_colour
{
public:
  explicit Init_ObjectOdometryPath_colour(::dynamic_slam_interfaces::msg::ObjectOdometryPath & msg)
  : msg_(msg)
  {}
  Init_ObjectOdometryPath_object_id colour(::dynamic_slam_interfaces::msg::ObjectOdometryPath::_colour_type arg)
  {
    msg_.colour = std::move(arg);
    return Init_ObjectOdometryPath_object_id(msg_);
  }

private:
  ::dynamic_slam_interfaces::msg::ObjectOdometryPath msg_;
};

class Init_ObjectOdometryPath_header
{
public:
  Init_ObjectOdometryPath_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ObjectOdometryPath_colour header(::dynamic_slam_interfaces::msg::ObjectOdometryPath::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_ObjectOdometryPath_colour(msg_);
  }

private:
  ::dynamic_slam_interfaces::msg::ObjectOdometryPath msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::dynamic_slam_interfaces::msg::ObjectOdometryPath>()
{
  return dynamic_slam_interfaces::msg::builder::Init_ObjectOdometryPath_header();
}

}  // namespace dynamic_slam_interfaces

#endif  // DYNAMIC_SLAM_INTERFACES__MSG__DETAIL__OBJECT_ODOMETRY_PATH__BUILDER_HPP_
