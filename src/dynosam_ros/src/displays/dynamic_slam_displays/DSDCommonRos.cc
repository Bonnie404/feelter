#include "dynosam_ros/displays/dynamic_slam_displays/DSDCommonRos.hpp"

#include <glog/logging.h>

#include <dynosam/common/DynamicObjects.hpp>
#include <dynosam/visualizer/Colour.hpp>

#include "dynosam_ros/RosUtils.hpp"
#include "dynosam_ros/displays/DisplaysCommon.hpp"

namespace dyno {

DSDTransport::DSDTransport(rclcpp::Node::SharedPtr node) : node_(node) {
  object_odom_publisher_ =
      node->create_publisher<ObjectOdometry>("object_odometry", 1);
  multi_object_odom_path_publisher_ =
      node->create_publisher<MultiObjectOdometryPath>("object_odometry_path",
                                                      1);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*node_);

  VLOG(20) << "Constructed DSDTransport with effective namespace "
           << node_->get_effective_namespace();
}

std::string DSDTransport::constructObjectFrameLink(ObjectId object_id) {
  return "object_" + std::to_string(object_id) + "_link";
}

ObjectOdometry DSDTransport::constructObjectOdometry(
    const gtsam::Pose3& motion_k, const gtsam::Pose3& pose_k,
    ObjectId object_id, Timestamp timestamp_k, const std::string& frame_id_link,
    const std::string& child_frame_id_link) {
  ObjectOdometry object_odom;

  // technically this shoudl be k-1
  gtsam::Point3 body_velocity = calculateBodyMotion(motion_k, pose_k);

  nav_msgs::msg::Odometry odom_msg;
  utils::convertWithHeader(pose_k, odom_msg, timestamp_k, frame_id_link,
                           child_frame_id_link);

  object_odom.odom = odom_msg;
  // TODO: can check if correct representation?

  dyno::convert(motion_k, object_odom.h_w_km1_k.pose);
  // NO velocity!!
  object_odom.object_id = object_id;
  return object_odom;
}

ObjectOdometryMap DSDTransport::constructObjectOdometries(
    const MotionEstimateMap& motions_k, const ObjectPoseMap& poses,
    FrameId frame_id_k, Timestamp timestamp_k,
    const std::string& frame_id_link) {
  // need to get poses for k-1
  // TODO: no way to ensure that the motions are for frame k
  // this is a weird data-structure to use and motions are per frame and
  // ObjectPoseMap is over all k to K
  //  const FrameId frame_id_k_1 = frame_id_k - 1u;
  ObjectOdometryMap object_odom_map;
  for (const auto& [object_id, object_motion] : motions_k) {
    const gtsam::Pose3& motion_k = object_motion;

    if (!poses.exists(object_id, frame_id_k)) {
      LOG(WARNING) << "Cannot construct ObjectOdometry for object " << object_id
                   << ", at frame " << frame_id_k
                   << " Missing entry in ObjectPoseMap";
      continue;
    }

    const gtsam::Pose3& pose_k = poses.at(object_id, frame_id_k);

    const std::string child_frame_id_link = constructObjectFrameLink(object_id);

    object_odom_map.insert2(
        child_frame_id_link,
        constructObjectOdometry(motion_k, pose_k, object_id, timestamp_k,
                                frame_id_link, child_frame_id_link));
  }

  return object_odom_map;
}

MultiObjectOdometryPath DSDTransport::constructMultiObjectOdometryPaths(
    const MotionEstimateMap& motions_k, const ObjectPoseMap& poses,
    FrameId frame_id_k, Timestamp timestamp_k, rclcpp::Time ros_time_now,
    const std::string& frame_id_link) {
  MultiObjectOdometryPath multi_path;
  multi_path.header.stamp = ros_time_now;
  multi_path.header.frame_id = frame_id_link;

  // TODO: right now dont have the motion for every timestep so... just leave
  // blank?
  for (const auto& [object_id, frame_pose_map] : poses) {
    const std::string child_frame_id_link = constructObjectFrameLink(object_id);
    FrameId previous_frame_id = -1;
    bool first = true;
    int path_segment = 0;

    std_msgs::msg::ColorRGBA colour_msg;
    convert(Color::uniqueId(object_id), colour_msg);

    // paths for this object, broken into segments
    gtsam::FastMap<int, ObjectOdometryPath> segmented_paths;

    for (const auto& [frame_id, object_pose] : frame_pose_map) {
      if (!first && frame_id != previous_frame_id + 1) {
        path_segment++;
      }
      first = false;
      previous_frame_id = frame_id;

      // RIGHT NOW MOTION IDENTITY
      // timestamp is wring
      gtsam::Pose3 motion;
      const ObjectOdometry object_odometry =
          constructObjectOdometry(motion, object_pose, object_id, timestamp_k,
                                  frame_id_link, child_frame_id_link);

      if (!segmented_paths.exists(path_segment)) {
        ObjectOdometryPath path;
        path.colour = colour_msg;
        path.object_id = object_id;
        path.path_segment = path_segment;

        path.header = multi_path.header;

        segmented_paths.insert2(path_segment, path);
      }

      ObjectOdometryPath& path = segmented_paths.at(path_segment);
      path.object_odometries.push_back(object_odometry);
    }

    for (const auto& [_, path] : segmented_paths) {
      multi_path.paths.push_back(path);
    }
  }

  return multi_path;
}

void DSDTransport::Publisher::publishObjectOdometry() {
  for (const auto& [_, object_odom] : object_odometries_)
    object_odom_publisher_->publish(object_odom);
}

void DSDTransport::Publisher::publishObjectTransforms() {
  for (const auto& [object_child_frame, object_odom] : object_odometries_) {
    geometry_msgs::msg::TransformStamped t;
    dyno::convert(object_odom.odom.pose.pose, t.transform);

    t.header.stamp = node_->now();
    t.header.frame_id = frame_id_link_;
    t.child_frame_id = object_child_frame;

    tf_broadcaster_->sendTransform(t);
  }
}

void DSDTransport::Publisher::publishObjectPaths() {
  multi_object_odom_path_publisher_->publish(object_paths_);
}

DSDTransport::Publisher::Publisher(
    rclcpp::Node::SharedPtr node,
    ObjectOdometryPub::SharedPtr object_odom_publisher,
    MultiObjectOdometryPathPub::SharedPtr multi_object_odom_path_publisher,
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster,
    const MotionEstimateMap& motions, const ObjectPoseMap& poses,
    const std::string& frame_id_link, FrameId frame_id, Timestamp timestamp)
    : node_(node),
      object_odom_publisher_(object_odom_publisher),
      multi_object_odom_path_publisher_(multi_object_odom_path_publisher),
      tf_broadcaster_(tf_broadcaster),
      frame_id_link_(frame_id_link),
      frame_id_(frame_id),
      timestamp_(timestamp),
      object_odometries_(DSDTransport::constructObjectOdometries(
          motions, poses, frame_id, timestamp, frame_id_link)),
      object_paths_(DSDTransport::constructMultiObjectOdometryPaths(
          motions, poses, frame_id, timestamp, node->now(), frame_id_link)) {}

DSDTransport::Publisher DSDTransport::addObjectInfo(
    const MotionEstimateMap& motions_k, const ObjectPoseMap& poses,
    const std::string& frame_id_link, FrameId frame_id, Timestamp timestamp) {
  return Publisher(node_, object_odom_publisher_,
                   multi_object_odom_path_publisher_, tf_broadcaster_,
                   motions_k, poses, frame_id_link, frame_id, timestamp);
}

DSDRos::DSDRos(const DisplayParams& params, rclcpp::Node::SharedPtr node)
    : params_(params), node_(node), dsd_transport_(node) {
  vo_publisher_ =
      node_->create_publisher<nav_msgs::msg::Odometry>("odometry", 1);
  vo_path_publisher_ =
      node_->create_publisher<nav_msgs::msg::Path>("odometry_path", 1);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*node_);

  static_points_pub_ =
      node->create_publisher<sensor_msgs::msg::PointCloud2>("static_cloud", 1);
  dynamic_points_pub_ =
      node->create_publisher<sensor_msgs::msg::PointCloud2>("dynamic_cloud", 1);
}

void DSDRos::publishVisualOdometry(const gtsam::Pose3& T_world_camera,
                                   Timestamp timestamp, const bool publish_tf) {
  DisplayCommon::publishOdometry(vo_publisher_, T_world_camera, timestamp,
                                 params_.world_frame_id,
                                 params_.camera_frame_id);

  if (publish_tf) {
    geometry_msgs::msg::TransformStamped t;
    dyno::convert<gtsam::Pose3, geometry_msgs::msg::TransformStamped>(
        T_world_camera, t);

    t.header.stamp = node_->now();
    t.header.frame_id = params_.world_frame_id;
    t.child_frame_id = params_.camera_frame_id;
    tf_broadcaster_->sendTransform(t);
  }
}
void DSDRos::publishVisualOdometryPath(const gtsam::Pose3Vector& poses,
                                       Timestamp latest_timestamp) {
  DisplayCommon::publishOdometryPath(vo_path_publisher_, poses,
                                     latest_timestamp, params_.world_frame_id);
}

CloudPerObject DSDRos::publishStaticPointCloud(
    const StatusLandmarkVector& landmarks, const gtsam::Pose3& T_world_camera) {
  return DisplayCommon::publishPointCloud(
      static_points_pub_, landmarks, T_world_camera, params_.world_frame_id);
}
CloudPerObject DSDRos::publishDynamicPointCloud(
    const StatusLandmarkVector& landmarks, const gtsam::Pose3& T_world_camera) {
  return DisplayCommon::publishPointCloud(
      dynamic_points_pub_, landmarks, T_world_camera, params_.world_frame_id);
}

}  // namespace dyno
