// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from dynamic_slam_interfaces:msg/ObjectOdometry.idl
// generated code does not contain a copyright notice

#ifndef DYNAMIC_SLAM_INTERFACES__MSG__DETAIL__OBJECT_ODOMETRY__BUILDER_HPP_
#define DYNAMIC_SLAM_INTERFACES__MSG__DETAIL__OBJECT_ODOMETRY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "dynamic_slam_interfaces/msg/detail/object_odometry__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace dynamic_slam_interfaces
{

namespace msg
{

namespace builder
{

class Init_ObjectOdometry_sequence
{
public:
  explicit Init_ObjectOdometry_sequence(::dynamic_slam_interfaces::msg::ObjectOdometry & msg)
  : msg_(msg)
  {}
  ::dynamic_slam_interfaces::msg::ObjectOdometry sequence(::dynamic_slam_interfaces::msg::ObjectOdometry::_sequence_type arg)
  {
    msg_.sequence = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dynamic_slam_interfaces::msg::ObjectOdometry msg_;
};

class Init_ObjectOdometry_object_id
{
public:
  explicit Init_ObjectOdometry_object_id(::dynamic_slam_interfaces::msg::ObjectOdometry & msg)
  : msg_(msg)
  {}
  Init_ObjectOdometry_sequence object_id(::dynamic_slam_interfaces::msg::ObjectOdometry::_object_id_type arg)
  {
    msg_.object_id = std::move(arg);
    return Init_ObjectOdometry_sequence(msg_);
  }

private:
  ::dynamic_slam_interfaces::msg::ObjectOdometry msg_;
};

class Init_ObjectOdometry_h_w_km1_k
{
public:
  explicit Init_ObjectOdometry_h_w_km1_k(::dynamic_slam_interfaces::msg::ObjectOdometry & msg)
  : msg_(msg)
  {}
  Init_ObjectOdometry_object_id h_w_km1_k(::dynamic_slam_interfaces::msg::ObjectOdometry::_h_w_km1_k_type arg)
  {
    msg_.h_w_km1_k = std::move(arg);
    return Init_ObjectOdometry_object_id(msg_);
  }

private:
  ::dynamic_slam_interfaces::msg::ObjectOdometry msg_;
};

class Init_ObjectOdometry_odom
{
public:
  Init_ObjectOdometry_odom()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ObjectOdometry_h_w_km1_k odom(::dynamic_slam_interfaces::msg::ObjectOdometry::_odom_type arg)
  {
    msg_.odom = std::move(arg);
    return Init_ObjectOdometry_h_w_km1_k(msg_);
  }

private:
  ::dynamic_slam_interfaces::msg::ObjectOdometry msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::dynamic_slam_interfaces::msg::ObjectOdometry>()
{
  return dynamic_slam_interfaces::msg::builder::Init_ObjectOdometry_odom();
}

}  // namespace dynamic_slam_interfaces

#endif  // DYNAMIC_SLAM_INTERFACES__MSG__DETAIL__OBJECT_ODOMETRY__BUILDER_HPP_
