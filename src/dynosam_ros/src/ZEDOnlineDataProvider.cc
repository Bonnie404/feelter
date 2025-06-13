#include "dynosam_ros/ZEDOnlineDataProvider.hpp"
#include "dynosam_ros/RosUtils.hpp"
#include "dynosam/dataprovider/DataInterfacePipeline.hpp"
#include "dynosam/common/ImageTypes.hpp"
#include <glog/logging.h>
#include <chrono>
#include <rclcpp/wait_for_message.hpp>


namespace dyno {
ZEDOnlineDataProvider::ZEDOnlineDataProvider(
    rclcpp::Node::SharedPtr ros_node,
    const ZEDOnlineDataProviderParams& params)
  : DataProvider(),
    ros_node_(ros_node),
    params_(params),
    imu_buffer_(params_.imu_buffer_size) {
  CHECK(ros_node_) << "ROS node pointer cannot be null for ZEDOnlineDataProvider.";

  LOG(INFO) << "[ZEDOnlineDataProvider] Initializing with topics:"
      << "\n  RGB: " << params_.rgb_topic_name
      << "\n  Depth: " << params_.depth_topic_name
      << "\n  Semantic Mask: " << params_.semantic_mask_topic_name
      << "\n  Optical Flow: " << params_.optical_flow_topic_name
      << "\n  Detected Objects: " << params_.detected_objects_topic_name
      << "\n  Skeletons: " << params_.skeleton_topic_name
      << "\n  CamInfo: " << params_.camera_info_topic_name
      << "\n  IMU: " << params_.imu_topic_name;

  // 1. CameraInfo Subscriber
  // Standard camera info QoS - compatible with most camera drivers
  auto camera_info_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
  camera_info_sub_ = ros_node_->create_subscription<
    sensor_msgs::msg::CameraInfo>(
      params_.camera_info_topic_name,
      camera_info_qos,
      std::bind(&ZEDOnlineDataProvider::cameraInfoCallback, this,
                std::placeholders::_1));
  VLOG(1) << "[ZEDOnlineDataProvider] Subscribed to CameraInfo: " <<
 camera_info_sub_->get_topic_name();

  // Wait for camera info
  if (params_.wait_for_camera_info) {
    VLOG(1) << "[ZEDOnlineDataProvider] Waiting for initial CameraInfo...";
    sensor_msgs::msg::CameraInfo camera_info_msg;
    bool received = false;

    if (params_.camera_info_timeout_ms.count() < 0) {
      // Wait indefinitely
      received = rclcpp::wait_for_message(
          camera_info_msg, ros_node_, params_.camera_info_topic_name);
    } else {
      // Wait with timeout
      received = rclcpp::wait_for_message(
          camera_info_msg, ros_node_, params_.camera_info_topic_name,
          params_.camera_info_timeout_ms);
    }

    if (received) {
      // Process the camera info
      CameraParams cam_params_temp;
      rclcpp::TypeAdapter<dyno::CameraParams,
                          sensor_msgs::msg::CameraInfo>::convert_to_custom(
          camera_info_msg, cam_params_temp);

      std::lock_guard<std::mutex> lock(camera_info_mutex_);
      camera_params_ = cam_params_temp;
      camera_info_received_.store(true);

      VLOG(1) << "[ZEDOnlineDataProvider] Initial CameraInfo received.";
      LOG_IF(INFO, VLOG_IS_ON(1)) <<
          "[ZEDOnlineDataProvider] Camera Params from ROS: " << camera_params_.
value().toString();
    } else {
      std::string err_msg = "Timeout waiting for CameraInfo on topic: " +
                            params_.camera_info_topic_name;
      throw std::runtime_error(err_msg);
    }
  }
  // 2. Image Subscribers
  rgb_sub_ = std::make_unique<image_transport::SubscriberFilter>(
      ros_node_.get(), params_.rgb_topic_name, "raw");
  depth_sub_ = std::make_unique<image_transport::SubscriberFilter>(
      ros_node_.get(), params_.depth_topic_name, "raw");
  semantic_mask_sub_ = std::make_unique<image_transport::SubscriberFilter>(
      ros_node_.get(), params_.semantic_mask_topic_name, "raw");

  // Optical flow uses regular message_filters subscriber as it's not a standard Image message
  optical_flow_sub_ = std::make_unique<message_filters::Subscriber<
    zed_msgs::msg::OpticalFlowStamped>>(
      ros_node_.get(), params_.optical_flow_topic_name);

  VLOG(1) << "[ZEDOnlineDataProvider] Created SubscriberFilter for RGB: " <<
 rgb_sub_->getTopic();
  VLOG(1) << "[ZEDOnlineDataProvider] Created SubscriberFilter for Depth: " <<
 depth_sub_->getTopic();
  VLOG(1) <<
 "[ZEDOnlineDataProvider] Created SubscriberFilter for Semantic Mask: " <<
 semantic_mask_sub_->getTopic();
  VLOG(1) << "[ZEDOnlineDataProvider] Created Subscriber for Optical Flow: " <<
 params_.optical_flow_topic_name;

  // 3. Synchronizer for RGB, Depth, Semantic Mask, and Optical Flow
  image_sync_ = std::make_unique<message_filters::Synchronizer<
    ZedImageSyncPolicy>>(
      ZedImageSyncPolicy(params_.image_sync_queue_size),
      *rgb_sub_,
      *depth_sub_,
      *semantic_mask_sub_,
      *optical_flow_sub_);
  image_sync_->setAgePenalty(0.01);
  // Lower values prefer newer messages in a pair
  image_sync_->setMaxIntervalDuration(
      rclcpp::Duration::from_seconds(params_.image_sync_slop_sec));
  image_sync_->registerCallback(std::bind(
      &ZEDOnlineDataProvider::imageSyncCallback,
      this, std::placeholders::_1, std::placeholders::_2,
      std::placeholders::_3, std::placeholders::_4));
  VLOG(1) <<
 "[ZEDOnlineDataProvider] Image synchronizer (RGB, Depth, Semantic, Optical Flow) configured with slop: "
 << params_.image_sync_slop_sec << "s.";

  // 4. IMU Subscriber (if enabled)
  if (params_.enable_imu_processing) {
    imu_sub_ = ros_node_->create_subscription<sensor_msgs::msg::Imu>(
        params_.imu_topic_name,
        rclcpp::SensorDataQoS(), // Standard QoS for sensor data
        std::bind(&ZEDOnlineDataProvider::imuCallback, this,
                  std::placeholders::_1));
    VLOG(1) << "[ZEDOnlineDataProvider] Subscribed to IMU: " << imu_sub_->
get_topic_name();
  } else {
    VLOG(1) <<
 "[ZEDOnlineDataProvider] IMU processing is disabled by ZEDOnlineDataProviderParams.";
  }

  // 5. Detected Objects Subscriber
  detected_objects_sub_ = ros_node_->create_subscription<
    zed_msgs::msg::ObjectsStamped>(
      params_.detected_objects_topic_name,
      rclcpp::SensorDataQoS(),
      std::bind(&ZEDOnlineDataProvider::detectedObjectsCallback, this,
                std::placeholders::_1));
  VLOG(1) << "[ZEDOnlineDataProvider] Subscribed to Detected Objects: " <<
 detected_objects_sub_->get_topic_name();

  // 6. Skeletons Subscriber
  detected_skeletons_sub_ = ros_node_->create_subscription<
    zed_msgs::msg::ObjectsStamped>(
      params_.skeleton_topic_name,
      rclcpp::SensorDataQoS(),
      std::bind(&ZEDOnlineDataProvider::detectedSkeletonsCallback, this,
                std::placeholders::_1));
  VLOG(1) << "[ZEDOnlineDataProvider] Subscribed to Skeletons: " <<
 detected_skeletons_sub_->get_topic_name();

  LOG(INFO) << "[ZEDOnlineDataProvider] Initialization successful.";
}

ZEDOnlineDataProvider::~ZEDOnlineDataProvider() {
  ZEDOnlineDataProvider::shutdown();
}

bool ZEDOnlineDataProvider::spin() {
  if (shutdown_.load()) {
    return false;
  }
  // Callbacks are handled by the executor of ros_node_ (the DynoNode).
  // This spin can just check ROS status.
  return rclcpp::ok();
}

void ZEDOnlineDataProvider::shutdown() {
  if (shutdown_.exchange(true)) {
    // Ensure shutdown logic runs only once
    return;
  }
  LOG(INFO) << "[ZEDOnlineDataProvider] Shutting down...";
  if (image_sync_) {
    image_sync_.reset();
  }
  if (rgb_sub_) rgb_sub_.reset(); // unique_ptr will handle destruction
  if (depth_sub_) depth_sub_.reset();
  if (semantic_mask_sub_) semantic_mask_sub_.reset();
  if (optical_flow_sub_) optical_flow_sub_.reset();

  if (camera_info_sub_) camera_info_sub_.reset();
  if (imu_sub_) imu_sub_.reset();
  if (detected_objects_sub_) detected_objects_sub_.reset();
  if (detected_skeletons_sub_) detected_skeletons_sub_.reset();

  DataProvider::shutdown(); // Call base class shutdown
  LOG(INFO) << "[ZEDOnlineDataProvider] Shutdown complete.";
}

CameraParams::Optional ZEDOnlineDataProvider::getCameraParams() const {
  std::lock_guard<std::mutex> lock(camera_info_mutex_);
  return camera_params_;
}

cv::Mat ZEDOnlineDataProvider::convertRosImageToCvMat(
    const sensor_msgs::msg::Image::ConstSharedPtr& img_msg,
    const std::string& expected_encoding_cv_bridge) {
  cv_bridge::CvImageConstPtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvShare(img_msg, expected_encoding_cv_bridge);
  } catch (const cv_bridge::Exception& e) {
    LOG(ERROR) <<
        "[ZEDOnlineDataProvider] cv_bridge exception converting ROS image (encoding: "
        << img_msg->encoding << ") to "
        << expected_encoding_cv_bridge << ": " << e.what();
    return {};
  }
  if (!cv_ptr) {
    LOG(ERROR) <<
        "[ZEDOnlineDataProvider] cv_bridge::toCvShare returned null for encoding "
        << expected_encoding_cv_bridge;
    return {};
  }
  // .clone() is important to ensure data ownership if the CvImageConstPtr goes out of scope
  // or if the underlying message buffer is reused.
  return cv_ptr->image.clone();
}

cv::Mat ZEDOnlineDataProvider::readRgbRosImage(
    const sensor_msgs::msg::Image::ConstSharedPtr& img_msg) const {
  std::string target_encoding = params_.output_rgb
                                  ? sensor_msgs::image_encodings::BGR8
                                  : sensor_msgs::image_encodings::MONO8;
  // cv_bridge will attempt conversion if img_msg->encoding is different but compatible (e.g., BGRA8 -> BGR8)
  return ZEDOnlineDataProvider::convertRosImageToCvMat(
      img_msg, target_encoding);
}

cv::Mat ZEDOnlineDataProvider::readDepthRosImage(
    const sensor_msgs::msg::Image::ConstSharedPtr& img_msg) const {
  cv_bridge::CvImageConstPtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvShare(img_msg); // Get original encoding first
  } catch (const cv_bridge::Exception& e) {
    LOG(ERROR) <<
        "[ZEDOnlineDataProvider] cv_bridge exception in readDepthRosImage (initial share): "
        << e.what();
    return {};
  }

  if (!cv_ptr) {
    LOG(ERROR) <<
        "[ZEDOnlineDataProvider] cv_bridge::toCvShare returned null for depth image.";
    return {};
  }

  cv::Mat depth_cv = cv_ptr->image;
  cv::Mat depth_dynosam;

  if (cv_ptr->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
    depth_cv.convertTo(depth_dynosam, CV_64F); // meters to meters
  } else if (cv_ptr->encoding == sensor_msgs::image_encodings::TYPE_16UC1) {
    depth_cv.convertTo(depth_dynosam, CV_64F, 1.0 / 1000.0);
    // millimeters to meters
  } else {
    LOG(ERROR) << "[ZEDOnlineDataProvider] Unsupported depth format: " <<
        utils::cvTypeToString(depth_cv.type())
        << " with ROS encoding " << cv_ptr->encoding
        << ". Expected TYPE_32FC1 (meters) or TYPE_16UC1 (mm).";
    return {};
  }
  return depth_dynosam;
}

inline const char* toString(ThreadsafeImuBuffer::QueryResult result) {
  switch (result) {
    case ThreadsafeImuBuffer::QueryResult::kDataAvailable:
      return "kDataAvailable";
    case ThreadsafeImuBuffer::QueryResult::kDataNotYetAvailable:
      return "kDataNotYetAvailable";
    case ThreadsafeImuBuffer::QueryResult::kDataNeverAvailable:
      return "kDataNeverAvailable";
    case ThreadsafeImuBuffer::QueryResult::kQueueShutdown:
      return "kQueueShutdown";
    case ThreadsafeImuBuffer::QueryResult::kTooFewMeasurementsAvailable:
      return "kTooFewMeasurementsAvailable";
    default:
      return "Unknown";
  }
}

void ZEDOnlineDataProvider::imageSyncCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr& rgb_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr& semantic_msg,
    const zed_msgs::msg::OpticalFlowStamped::ConstSharedPtr& optical_flow_msg) {
  if (shutdown_.load() || !camera_info_received_.load()) {
    VLOG_EVERY_N(1, 100) <<
 "[ZEDOnlineDataProvider] Skipping image sync: shutdown=" << shutdown_.load()
                             << ", camera_info_received=" <<
 camera_info_received_.load();
    return;
  }
  CHECK(camera_params_.has_value()) <<
 "[ZEDOnlineDataProvider] CameraInfo not received but processing images.";

  Timestamp current_timestamp_s = utils::fromRosTime(rgb_msg->header.stamp);
  Timestamp current_timestamp_ns =
      static_cast<Timestamp>(rgb_msg->header.stamp.sec) * 1000000000ULL +
      rgb_msg->header.stamp.nanosec;

  VLOG(2) << "[ZEDOnlineDataProvider] Synced Images Received: Frame " <<
 frame_id_counter_
            << " RGB TS: " << std::fixed << std::setprecision(3) <<
 utils::fromRosTime(rgb_msg->header.stamp)
            << " Depth TS: " << utils::fromRosTime(depth_msg->header.stamp);

  cv::Mat rgb_cv = readRgbRosImage(rgb_msg);
  cv::Mat depth_cv = readDepthRosImage(depth_msg);
  cv::Mat semantic_cv = readSemanticSegmentationRosImage(semantic_msg);
  cv::Mat optical_flow_cv = readOpticalFlowRos(optical_flow_msg);

  if (rgb_cv.empty() || depth_cv.empty()) {
    LOG(WARNING) <<
        "[ZEDOnlineDataProvider] Failed to convert RGB or Depth image for frame "
        << frame_id_counter_
        << ". RGB empty: " << rgb_cv.empty() << ", Depth empty: " << depth_cv.
        empty();
    return;
  }

  // Semantic and optical flow can be empty - they're optional
  if (semantic_cv.empty()) {
    VLOG(2) << "[ZEDOnlineDataProvider] Semantic mask is empty for frame " <<
 frame_id_counter_;
  }
  if (optical_flow_cv.empty()) {
    VLOG(2) << "[ZEDOnlineDataProvider] Optical flow is empty for frame " <<
 frame_id_counter_;
  }

  ImageContainer::Ptr image_container = ImageContainer::Create(
      current_timestamp_s,
      frame_id_counter_,
      ImageWrapper<ImageType::RGBMono>(rgb_cv),
      ImageWrapper<ImageType::Depth>(depth_cv),
      ImageWrapper<ImageType::OpticalFlow>(optical_flow_cv),
      ImageWrapper<ImageType::SemanticMask>(semantic_cv)
      );

  CHECK(image_container_callback_) <<
 "[ZEDOnlineDataProvider] Image container callback not registered!";
  image_container_callback_(image_container);

  if (params_.enable_imu_processing && (
        imu_single_input_callback_ || imu_multi_input_callback_)) {
    ImuMeasurements imu_measurements_for_frontend;
    if (last_image_timestamp_ns_ != 0 && current_timestamp_ns >
        last_image_timestamp_ns_) {
      ThreadsafeImuBuffer::QueryResult query_res = imu_buffer_.
          getImuDataInterpolatedBorders(
              last_image_timestamp_ns_,
              current_timestamp_ns,
              &imu_measurements_for_frontend.timestamps_,
              &imu_measurements_for_frontend.acc_gyr_);

      if (query_res == ThreadsafeImuBuffer::QueryResult::kDataAvailable &&
          imu_measurements_for_frontend.timestamps_.cols() > 0) {
        VLOG(3) << "[ZEDOnlineDataProvider] Retrieved " <<
 imu_measurements_for_frontend.timestamps_.cols()
                        << " IMU samples between " << last_image_timestamp_ns_
                        << " and " << current_timestamp_ns;
        if (imu_multi_input_callback_) {
          imu_multi_input_callback_(imu_measurements_for_frontend);
        } else if (imu_single_input_callback_) {
          // Fallback if only single callback is registered
          for (int i = 0; i < imu_measurements_for_frontend.timestamps_.cols();
               ++i) {
            ImuMeasurement single_imu;
            single_imu.timestamp_ =
                static_cast<Timestamp>(imu_measurements_for_frontend.
                  timestamps_(0, i)) * 1e-9; // Convert ns to s
            single_imu.acc_gyr_ = imu_measurements_for_frontend.acc_gyr_.col(i);
            imu_single_input_callback_(single_imu);
          }
        }
      } else {
        VLOG(3) <<
 "[ZEDOnlineDataProvider] Not enough IMU data or query failed for range "
                        << last_image_timestamp_ns_ << " - " <<
 current_timestamp_ns
                        << ". Query result: " << toString(query_res);
      }
    } else if (last_image_timestamp_ns_ == 0) {
      VLOG(2) << "[ZEDOnlineDataProvider] First image frame (ID: " <<
 frame_id_counter_ << "), no previous IMU data to query against yet.";
    } else if (current_timestamp_ns <= last_image_timestamp_ns_) {
      LOG(WARNING) << "[ZEDOnlineDataProvider] Current image timestamp (" <<
          current_timestamp_ns
          << ") is not after last image timestamp (" << last_image_timestamp_ns_
          << "). IMU data might be out of order or image timestamps duplicated. Frame ID: "
          << frame_id_counter_;
    }
    last_image_timestamp_ns_ = current_timestamp_ns;
  }

  frame_id_counter_++;
}

void ZEDOnlineDataProvider::cameraInfoCallback(
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info_msg) {
  if (camera_info_received_.load() && !params_.wait_for_camera_info) {
    VLOG(2) << "[ZEDOnlineDataProvider] Ignoring subsequent CameraInfo message.";
    return;
  }
  VLOG(1) << "[ZEDOnlineDataProvider] Received CameraInfo on topic: " <<
 info_msg->header.frame_id
            << " at " << utils::fromRosTime(info_msg->header.stamp);

  CameraParams cam_params_dyno_temp;
  rclcpp::TypeAdapter<dyno::CameraParams,
                      sensor_msgs::msg::CameraInfo>::convert_to_custom(
      *info_msg, cam_params_dyno_temp);

  {
    std::lock_guard<std::mutex> lock(camera_info_mutex_);
    camera_params_ = cam_params_dyno_temp;
    if (!camera_info_received_.load()) {
      camera_info_received_.store(true);
      VLOG(1) <<
 "[ZEDOnlineDataProvider] CameraParams successfully parsed and stored.";
      LOG_IF(INFO, VLOG_IS_ON(1)) <<
 "[ZEDOnlineDataProvider] Camera Params from ROS: " << camera_params_.value().
toString();
    }
  }
  if (params_.wait_for_camera_info) camera_info_cond_.notify_all();
}

void ZEDOnlineDataProvider::imuCallback(
    const sensor_msgs::msg::Imu::ConstSharedPtr& imu_msg) {
  if (shutdown_.load() || !params_.enable_imu_processing) return;

  Timestamp timestamp_ns = static_cast<Timestamp>(imu_msg->header.stamp.sec) *
                           1000000000ULL +
                           imu_msg->header.stamp.nanosec;
  ImuAccGyr acc_gyr_ros_frame;
  acc_gyr_ros_frame << imu_msg->linear_acceleration.x,
      imu_msg->linear_acceleration.y,
      imu_msg->linear_acceleration.z,
      imu_msg->angular_velocity.x,
      imu_msg->angular_velocity.y,
      imu_msg->angular_velocity.z;

  // IMPORTANT: Coordinate Frame Transformation for IMU Data
  // The zed-ros2-wrapper publishes IMU data in the frame defined by its URDF, typically `<camera_name>_imu_link`.
  // This frame is usually FLU (X-Forward, Y-Left, Z-Up) by ROS convention if the URDF is standard.
  // DynoSAM's core IMU processing might expect a specific convention (e.g., Body Frame: X-Forward, Y-Left, Z-Up, or
  //                                                                        Sensor Frame: X-Right, Y-Down, Z-Forward).
  // If DynoSAM expects IMU data in a frame other than `<camera_name>_imu_link` (e.g., a `base_link` frame),
  // and it does not handle this transformation internally using TF, then that transformation
  // should ideally happen *before* this DataProvider, or this DataProvider would need TF lookup capabilities.
  // For axis convention mismatch (e.g. `_imu_link` is NED, DynoSAM expects FLU), transform here:
  ImuAccGyr acc_gyr_dynosam_convention = acc_gyr_ros_frame;
  // Default: Assume direct use or TF handles it.

  // Example transformation if ZED's `_imu_link` was publishing in NED (X Fwd, Y Right, Z Down)
  // and DynOSAM's `ImuAccGyr` expects FLU (X Fwd, Y Left, Z Up):
  // acc_gyr_dynosam_convention(0) =  acc_gyr_ros_frame(0);  // AccX_flu = AccX_ned
  // acc_gyr_dynosam_convention(1) = -acc_gyr_ros_frame(1);  // AccY_flu = -AccY_ned (Right -> Left)
  // acc_gyr_dynosam_convention(2) = -acc_gyr_ros_frame(2);  // AccZ_flu = -AccZ_ned (Down -> Up)
  // acc_gyr_dynosam_convention(3) =  acc_gyr_ros_frame(3);  // GyroX_flu = GyroX_ned
  // acc_gyr_dynosam_convention(4) = -acc_gyr_ros_frame(4);  // GyroY_flu = -GyroY_ned
  // acc_gyr_dynosam_convention(5) = -acc_gyr_ros_frame(5);  // GyroZ_flu = -GyroZ_ned

  // This transformation needs to be confirmed based on ZED's actual IMU publishing convention
  // and DynOSAM's expectation. The zed-ros2-wrapper source (zed_camera_component.cpp publishSensorsData)
  // seems to publish raw SDK IMU data stamped with mImuFrameId. The SDK data is usually in
  // ZED's IMU sensor frame (often NED or similar). The URDF transform from _camera_center to _imu_link
  // would then define _imu_link in the robot's TF tree.
  // For now, assume zed-ros2-wrapper correctly publishes in a ROS-standard FLU frame for _imu_link.

  imu_buffer_.addMeasurement(timestamp_ns, acc_gyr_dynosam_convention);
}

void ZEDOnlineDataProvider::detectedObjectsCallback(
    const zed_msgs::msg::ObjectsStamped::ConstSharedPtr& objects_msg) {
  if (shutdown_.load()) return;

  std::lock_guard<std::mutex> lock(objects_mutex_);
  latest_detected_objects_ = std::make_shared<zed_msgs::msg::ObjectsStamped>(
      *objects_msg);

  VLOG(3) << "[ZEDOnlineDataProvider] Received " << objects_msg->objects.size()
          << " detected objects at timestamp " << utils::fromRosTime(
             objects_msg->header.stamp);
}

void ZEDOnlineDataProvider::detectedSkeletonsCallback(
    const zed_msgs::msg::ObjectsStamped::ConstSharedPtr& skeletons_msg) {
  if (shutdown_.load()) return;

  std::lock_guard<std::mutex> lock(objects_mutex_);
  latest_detected_skeletons_ = std::make_shared<zed_msgs::msg::ObjectsStamped>(
      *skeletons_msg);

  VLOG(3) << "[ZEDOnlineDataProvider] Received " << skeletons_msg->objects.
size()
          << " skeletons at timestamp " << utils::fromRosTime(
             skeletons_msg->header.stamp);
}


cv::Mat ZEDOnlineDataProvider::readOpticalFlowRos(
    const zed_msgs::msg::OpticalFlowStamped::ConstSharedPtr& optical_flow_msg)
const {
  if (!optical_flow_msg) {
    return cv::Mat();
  }

  // Convert the flow image from the message
  cv_bridge::CvImageConstPtr cv_ptr;
  try {
    // The optical flow image is in 32FC2 format (2 channels of float32 for x,y motion)
    // Create a shared pointer from the flow_image field
    sensor_msgs::msg::Image::ConstSharedPtr flow_img_ptr =
        std::make_shared<sensor_msgs::msg::Image>(optical_flow_msg->flow_image);
    cv_ptr = cv_bridge::toCvShare(flow_img_ptr,
                                  sensor_msgs::image_encodings::TYPE_32FC2);
  } catch (const cv_bridge::Exception& e) {
    LOG(ERROR) <<
        "[ZEDOnlineDataProvider] cv_bridge exception converting optical flow: "
        << e.what();
    return cv::Mat();
  }

  if (!cv_ptr) {
    LOG(ERROR) <<
        "[ZEDOnlineDataProvider] cv_bridge::toCvShare returned null for optical flow";
    return cv::Mat();
  }

  return cv_ptr->image.clone();
}

cv::Mat ZEDOnlineDataProvider::readSemanticSegmentationRosImage(
    const sensor_msgs::msg::Image::ConstSharedPtr& img_msg) const {
  if (!img_msg) {
    return cv::Mat();
  }

  // Semantic segmentation is a single channel image with class IDs
  cv_bridge::CvImageConstPtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvShare(img_msg);
  } catch (const cv_bridge::Exception& e) {
    LOG(ERROR) <<
        "[ZEDOnlineDataProvider] cv_bridge exception converting semantic mask: "
        << e.what();
    return cv::Mat();
  }

  if (!cv_ptr) {
    LOG(ERROR) <<
        "[ZEDOnlineDataProvider] cv_bridge::toCvShare returned null for semantic mask";
    return cv::Mat();
  }

  cv::Mat semantic_cv = cv_ptr->image;

  VLOG(3) << "[ZEDOnlineDataProvider] Semantic mask type: " <<
 utils::cvTypeToString(semantic_cv.type())
          << " with encoding " << cv_ptr->encoding;

  return semantic_cv.clone();
}
} // namespace dyno
