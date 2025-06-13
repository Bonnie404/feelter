// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from dynamic_slam_interfaces:msg/ObjectOdometryPath.idl
// generated code does not contain a copyright notice

#ifndef DYNAMIC_SLAM_INTERFACES__MSG__DETAIL__OBJECT_ODOMETRY_PATH__STRUCT_HPP_
#define DYNAMIC_SLAM_INTERFACES__MSG__DETAIL__OBJECT_ODOMETRY_PATH__STRUCT_HPP_

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
// Member 'colour'
#include "std_msgs/msg/detail/color_rgba__struct.hpp"
// Member 'object_odometries'
#include "dynamic_slam_interfaces/msg/detail/object_odometry__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__dynamic_slam_interfaces__msg__ObjectOdometryPath __attribute__((deprecated))
#else
# define DEPRECATED__dynamic_slam_interfaces__msg__ObjectOdometryPath __declspec(deprecated)
#endif

namespace dynamic_slam_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ObjectOdometryPath_
{
  using Type = ObjectOdometryPath_<ContainerAllocator>;

  explicit ObjectOdometryPath_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    colour(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::DEFAULTS_ONLY == _init)
    {
      this->path_segment = 0ll;
    } else if (rosidl_runtime_cpp::MessageInitialization::ZERO == _init) {
      this->object_id = 0ll;
      this->path_segment = 0ll;
    }
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->object_id = 0ll;
    }
  }

  explicit ObjectOdometryPath_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    colour(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::DEFAULTS_ONLY == _init)
    {
      this->path_segment = 0ll;
    } else if (rosidl_runtime_cpp::MessageInitialization::ZERO == _init) {
      this->object_id = 0ll;
      this->path_segment = 0ll;
    }
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->object_id = 0ll;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _colour_type =
    std_msgs::msg::ColorRGBA_<ContainerAllocator>;
  _colour_type colour;
  using _object_id_type =
    int64_t;
  _object_id_type object_id;
  using _path_segment_type =
    int64_t;
  _path_segment_type path_segment;
  using _object_odometries_type =
    std::vector<dynamic_slam_interfaces::msg::ObjectOdometry_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<dynamic_slam_interfaces::msg::ObjectOdometry_<ContainerAllocator>>>;
  _object_odometries_type object_odometries;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__colour(
    const std_msgs::msg::ColorRGBA_<ContainerAllocator> & _arg)
  {
    this->colour = _arg;
    return *this;
  }
  Type & set__object_id(
    const int64_t & _arg)
  {
    this->object_id = _arg;
    return *this;
  }
  Type & set__path_segment(
    const int64_t & _arg)
  {
    this->path_segment = _arg;
    return *this;
  }
  Type & set__object_odometries(
    const std::vector<dynamic_slam_interfaces::msg::ObjectOdometry_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<dynamic_slam_interfaces::msg::ObjectOdometry_<ContainerAllocator>>> & _arg)
  {
    this->object_odometries = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    dynamic_slam_interfaces::msg::ObjectOdometryPath_<ContainerAllocator> *;
  using ConstRawPtr =
    const dynamic_slam_interfaces::msg::ObjectOdometryPath_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<dynamic_slam_interfaces::msg::ObjectOdometryPath_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<dynamic_slam_interfaces::msg::ObjectOdometryPath_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      dynamic_slam_interfaces::msg::ObjectOdometryPath_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<dynamic_slam_interfaces::msg::ObjectOdometryPath_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      dynamic_slam_interfaces::msg::ObjectOdometryPath_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<dynamic_slam_interfaces::msg::ObjectOdometryPath_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<dynamic_slam_interfaces::msg::ObjectOdometryPath_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<dynamic_slam_interfaces::msg::ObjectOdometryPath_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__dynamic_slam_interfaces__msg__ObjectOdometryPath
    std::shared_ptr<dynamic_slam_interfaces::msg::ObjectOdometryPath_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__dynamic_slam_interfaces__msg__ObjectOdometryPath
    std::shared_ptr<dynamic_slam_interfaces::msg::ObjectOdometryPath_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ObjectOdometryPath_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->colour != other.colour) {
      return false;
    }
    if (this->object_id != other.object_id) {
      return false;
    }
    if (this->path_segment != other.path_segment) {
      return false;
    }
    if (this->object_odometries != other.object_odometries) {
      return false;
    }
    return true;
  }
  bool operator!=(const ObjectOdometryPath_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ObjectOdometryPath_

// alias to use template instance with default allocator
using ObjectOdometryPath =
  dynamic_slam_interfaces::msg::ObjectOdometryPath_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace dynamic_slam_interfaces

#endif  // DYNAMIC_SLAM_INTERFACES__MSG__DETAIL__OBJECT_ODOMETRY_PATH__STRUCT_HPP_
