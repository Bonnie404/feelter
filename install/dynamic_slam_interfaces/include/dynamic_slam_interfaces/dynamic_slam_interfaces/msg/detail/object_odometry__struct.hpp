// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from dynamic_slam_interfaces:msg/ObjectOdometry.idl
// generated code does not contain a copyright notice

#ifndef DYNAMIC_SLAM_INTERFACES__MSG__DETAIL__OBJECT_ODOMETRY__STRUCT_HPP_
#define DYNAMIC_SLAM_INTERFACES__MSG__DETAIL__OBJECT_ODOMETRY__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'odom'
#include "nav_msgs/msg/detail/odometry__struct.hpp"
// Member 'h_w_km1_k'
#include "geometry_msgs/msg/detail/pose_with_covariance__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__dynamic_slam_interfaces__msg__ObjectOdometry __attribute__((deprecated))
#else
# define DEPRECATED__dynamic_slam_interfaces__msg__ObjectOdometry __declspec(deprecated)
#endif

namespace dynamic_slam_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ObjectOdometry_
{
  using Type = ObjectOdometry_<ContainerAllocator>;

  explicit ObjectOdometry_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : odom(_init),
    h_w_km1_k(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->object_id = 0ll;
      this->sequence = 0ul;
    }
  }

  explicit ObjectOdometry_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : odom(_alloc, _init),
    h_w_km1_k(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->object_id = 0ll;
      this->sequence = 0ul;
    }
  }

  // field types and members
  using _odom_type =
    nav_msgs::msg::Odometry_<ContainerAllocator>;
  _odom_type odom;
  using _h_w_km1_k_type =
    geometry_msgs::msg::PoseWithCovariance_<ContainerAllocator>;
  _h_w_km1_k_type h_w_km1_k;
  using _object_id_type =
    int64_t;
  _object_id_type object_id;
  using _sequence_type =
    uint32_t;
  _sequence_type sequence;

  // setters for named parameter idiom
  Type & set__odom(
    const nav_msgs::msg::Odometry_<ContainerAllocator> & _arg)
  {
    this->odom = _arg;
    return *this;
  }
  Type & set__h_w_km1_k(
    const geometry_msgs::msg::PoseWithCovariance_<ContainerAllocator> & _arg)
  {
    this->h_w_km1_k = _arg;
    return *this;
  }
  Type & set__object_id(
    const int64_t & _arg)
  {
    this->object_id = _arg;
    return *this;
  }
  Type & set__sequence(
    const uint32_t & _arg)
  {
    this->sequence = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    dynamic_slam_interfaces::msg::ObjectOdometry_<ContainerAllocator> *;
  using ConstRawPtr =
    const dynamic_slam_interfaces::msg::ObjectOdometry_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<dynamic_slam_interfaces::msg::ObjectOdometry_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<dynamic_slam_interfaces::msg::ObjectOdometry_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      dynamic_slam_interfaces::msg::ObjectOdometry_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<dynamic_slam_interfaces::msg::ObjectOdometry_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      dynamic_slam_interfaces::msg::ObjectOdometry_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<dynamic_slam_interfaces::msg::ObjectOdometry_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<dynamic_slam_interfaces::msg::ObjectOdometry_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<dynamic_slam_interfaces::msg::ObjectOdometry_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__dynamic_slam_interfaces__msg__ObjectOdometry
    std::shared_ptr<dynamic_slam_interfaces::msg::ObjectOdometry_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__dynamic_slam_interfaces__msg__ObjectOdometry
    std::shared_ptr<dynamic_slam_interfaces::msg::ObjectOdometry_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ObjectOdometry_ & other) const
  {
    if (this->odom != other.odom) {
      return false;
    }
    if (this->h_w_km1_k != other.h_w_km1_k) {
      return false;
    }
    if (this->object_id != other.object_id) {
      return false;
    }
    if (this->sequence != other.sequence) {
      return false;
    }
    return true;
  }
  bool operator!=(const ObjectOdometry_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ObjectOdometry_

// alias to use template instance with default allocator
using ObjectOdometry =
  dynamic_slam_interfaces::msg::ObjectOdometry_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace dynamic_slam_interfaces

#endif  // DYNAMIC_SLAM_INTERFACES__MSG__DETAIL__OBJECT_ODOMETRY__STRUCT_HPP_
