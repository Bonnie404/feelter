// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from dynamic_slam_interfaces:msg/MultiObjectOdometryPath.idl
// generated code does not contain a copyright notice

#ifndef DYNAMIC_SLAM_INTERFACES__MSG__DETAIL__MULTI_OBJECT_ODOMETRY_PATH__STRUCT_HPP_
#define DYNAMIC_SLAM_INTERFACES__MSG__DETAIL__MULTI_OBJECT_ODOMETRY_PATH__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"
// Member 'paths'
#include "dynamic_slam_interfaces/msg/detail/object_odometry_path__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__dynamic_slam_interfaces__msg__MultiObjectOdometryPath __attribute__((deprecated))
#else
# define DEPRECATED__dynamic_slam_interfaces__msg__MultiObjectOdometryPath __declspec(deprecated)
#endif

namespace dynamic_slam_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MultiObjectOdometryPath_
{
  using Type = MultiObjectOdometryPath_<ContainerAllocator>;

  explicit MultiObjectOdometryPath_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    (void)_init;
  }

  explicit MultiObjectOdometryPath_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _paths_type =
    std::vector<dynamic_slam_interfaces::msg::ObjectOdometryPath_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<dynamic_slam_interfaces::msg::ObjectOdometryPath_<ContainerAllocator>>>;
  _paths_type paths;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__paths(
    const std::vector<dynamic_slam_interfaces::msg::ObjectOdometryPath_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<dynamic_slam_interfaces::msg::ObjectOdometryPath_<ContainerAllocator>>> & _arg)
  {
    this->paths = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    dynamic_slam_interfaces::msg::MultiObjectOdometryPath_<ContainerAllocator> *;
  using ConstRawPtr =
    const dynamic_slam_interfaces::msg::MultiObjectOdometryPath_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<dynamic_slam_interfaces::msg::MultiObjectOdometryPath_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<dynamic_slam_interfaces::msg::MultiObjectOdometryPath_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      dynamic_slam_interfaces::msg::MultiObjectOdometryPath_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<dynamic_slam_interfaces::msg::MultiObjectOdometryPath_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      dynamic_slam_interfaces::msg::MultiObjectOdometryPath_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<dynamic_slam_interfaces::msg::MultiObjectOdometryPath_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<dynamic_slam_interfaces::msg::MultiObjectOdometryPath_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<dynamic_slam_interfaces::msg::MultiObjectOdometryPath_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__dynamic_slam_interfaces__msg__MultiObjectOdometryPath
    std::shared_ptr<dynamic_slam_interfaces::msg::MultiObjectOdometryPath_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__dynamic_slam_interfaces__msg__MultiObjectOdometryPath
    std::shared_ptr<dynamic_slam_interfaces::msg::MultiObjectOdometryPath_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MultiObjectOdometryPath_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->paths != other.paths) {
      return false;
    }
    return true;
  }
  bool operator!=(const MultiObjectOdometryPath_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MultiObjectOdometryPath_

// alias to use template instance with default allocator
using MultiObjectOdometryPath =
  dynamic_slam_interfaces::msg::MultiObjectOdometryPath_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace dynamic_slam_interfaces

#endif  // DYNAMIC_SLAM_INTERFACES__MSG__DETAIL__MULTI_OBJECT_ODOMETRY_PATH__STRUCT_HPP_
