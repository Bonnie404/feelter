// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from zed_msgs:msg/OpticalFlowStamped.idl
// generated code does not contain a copyright notice

#ifndef ZED_MSGS__MSG__DETAIL__OPTICAL_FLOW_STAMPED__STRUCT_HPP_
#define ZED_MSGS__MSG__DETAIL__OPTICAL_FLOW_STAMPED__STRUCT_HPP_

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
// Member 'flow_image'
#include "sensor_msgs/msg/detail/image__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__zed_msgs__msg__OpticalFlowStamped __attribute__((deprecated))
#else
# define DEPRECATED__zed_msgs__msg__OpticalFlowStamped __declspec(deprecated)
#endif

namespace zed_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct OpticalFlowStamped_
{
  using Type = OpticalFlowStamped_<ContainerAllocator>;

  explicit OpticalFlowStamped_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    flow_image(_init)
  {
    (void)_init;
  }

  explicit OpticalFlowStamped_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    flow_image(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _flow_image_type =
    sensor_msgs::msg::Image_<ContainerAllocator>;
  _flow_image_type flow_image;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__flow_image(
    const sensor_msgs::msg::Image_<ContainerAllocator> & _arg)
  {
    this->flow_image = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    zed_msgs::msg::OpticalFlowStamped_<ContainerAllocator> *;
  using ConstRawPtr =
    const zed_msgs::msg::OpticalFlowStamped_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<zed_msgs::msg::OpticalFlowStamped_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<zed_msgs::msg::OpticalFlowStamped_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      zed_msgs::msg::OpticalFlowStamped_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<zed_msgs::msg::OpticalFlowStamped_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      zed_msgs::msg::OpticalFlowStamped_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<zed_msgs::msg::OpticalFlowStamped_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<zed_msgs::msg::OpticalFlowStamped_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<zed_msgs::msg::OpticalFlowStamped_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__zed_msgs__msg__OpticalFlowStamped
    std::shared_ptr<zed_msgs::msg::OpticalFlowStamped_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__zed_msgs__msg__OpticalFlowStamped
    std::shared_ptr<zed_msgs::msg::OpticalFlowStamped_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const OpticalFlowStamped_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->flow_image != other.flow_image) {
      return false;
    }
    return true;
  }
  bool operator!=(const OpticalFlowStamped_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct OpticalFlowStamped_

// alias to use template instance with default allocator
using OpticalFlowStamped =
  zed_msgs::msg::OpticalFlowStamped_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace zed_msgs

#endif  // ZED_MSGS__MSG__DETAIL__OPTICAL_FLOW_STAMPED__STRUCT_HPP_
