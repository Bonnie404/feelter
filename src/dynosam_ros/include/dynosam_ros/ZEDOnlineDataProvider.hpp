#pragma once

#include <dynosam/dataprovider/DataProvider.hpp> // From dynosam core
#include <dynosam/common/CameraParams.hpp>       // From dynosam core
#include <dynosam/frontend/imu/ThreadSafeImuBuffer.hpp> // From dynosam core
#include <dynosam/common/ImageTypes.hpp>        // For ImageWrapper

// ROS2 includes
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <zed_msgs/msg/objects_stamped.hpp>
#include <zed_msgs/msg/optical_flow_stamped.hpp>

#include <image_transport/image_transport.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <cv_bridge/cv_bridge.h>

// Local adaptor (within dynosam_ros)
#include "dynosam_ros/adaptors/CameraParamsAdaptor.hpp"

namespace dyno {

// Forward declare DataInterfacePipeline and DataInterfacePipelineIm
// as they are part of the dynosam core library this DataProvider interacts with.
class DataInterfacePipeline;
class DataInterfacePipelineImu;

struct ZEDOnlineDataProviderParams {
    std::string rgb_topic_name = "rgb/image_rect_color";
    std::string depth_topic_name = "depth/depth_registered";
    std::string optical_flow_topic_name = "optical_flow/image";
    std::string semantic_mask_topic_name = "semantic_mask/image";
    std::string detected_objects_topic_name = "detected_objects";
    std::string skeleton_topic_name = "skeletons";
    std::string camera_info_topic_name = "left/camera_info";
    std::string imu_topic_name = "imu/data";

    int image_sync_queue_size = 15;
    double image_sync_slop_sec = 0.034; // Approx 1 frame at 30fps

    bool wait_for_camera_info = true;
    std::chrono::milliseconds camera_info_timeout_ms = std::chrono::milliseconds(-1); // Wait indefinitely

    size_t imu_buffer_size = 200;
    bool output_rgb = true; // True for BGR, false for Grayscale
    bool enable_imu_processing = true; // To globally enable/disable IMU part of this provider

    // Transformation for IMU data from ZED's IMU frame to DynOSAM's expected body/IMU convention

};

class ZEDOnlineDataProvider : public DataProvider {
public:
    DYNO_POINTER_TYPEDEFS(ZEDOnlineDataProvider);

    ZEDOnlineDataProvider(
        rclcpp::Node::SharedPtr ros_node, // The ROS node (e.g., DynoNode instance) that owns this
        const ZEDOnlineDataProviderParams& params
    );

    ~ZEDOnlineDataProvider() override;

    // --- Overrides from DataProvider ---
    bool spin() override;
    void shutdown() override;
    CameraParams::Optional getCameraParams() const override;
    int datasetSize() const override { return -1; } // Online stream
    
    // --- Additional getters for ZED-specific data ---
    zed_msgs::msg::ObjectsStamped::SharedPtr getLatestDetectedObjects() const {
        std::lock_guard<std::mutex> lock(objects_mutex_);
        return latest_detected_objects_;
    }
    
    zed_msgs::msg::ObjectsStamped::SharedPtr getLatestDetectedSkeletons() const {
        std::lock_guard<std::mutex> lock(objects_mutex_);
        return latest_detected_skeletons_;
    }

private:
    void imageSyncCallback(
        const sensor_msgs::msg::Image::ConstSharedPtr& rgb_msg,
        const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg,
        const sensor_msgs::msg::Image::ConstSharedPtr& semantic_msg,
        const zed_msgs::msg::OpticalFlowStamped::ConstSharedPtr& optical_flow_msg);

    void cameraInfoCallback(
        const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info_msg);

    void imuCallback(
        const sensor_msgs::msg::Imu::ConstSharedPtr& imu_msg);

    void detectedObjectsCallback(
      const zed_msgs::msg::ObjectsStamped::ConstSharedPtr& objects_msg);

    void detectedSkeletonsCallback(
      const zed_msgs::msg::ObjectsStamped::ConstSharedPtr& skeletons_msg);

    void semanticSegmentationCallback(
      const sensor_msgs::msg::Image::ConstSharedPtr& semantic_msg
    );



    static cv::Mat convertRosImageToCvMat(
        const sensor_msgs::msg::Image::ConstSharedPtr& img_msg,
        const std::string& expected_encoding_cv_bridge);
    cv::Mat readRgbRosImage(const sensor_msgs::msg::Image::ConstSharedPtr& img_msg) const;
    cv::Mat readDepthRosImage(const sensor_msgs::msg::Image::ConstSharedPtr& img_msg) const;
    cv::Mat readOpticalFlowRos(const zed_msgs::msg::OpticalFlowStamped::ConstSharedPtr& optical_flow_msg) const;
    cv::Mat readSemanticSegmentationRosImage(const sensor_msgs::msg::Image::ConstSharedPtr& img_msg) const;

    rclcpp::Node::SharedPtr ros_node_;
    ZEDOnlineDataProviderParams params_;

    // Image synchronization (RGB + Depth + Semantic + Optical Flow)
    typedef message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::Image,             // RGB
        sensor_msgs::msg::Image,             // Depth
        sensor_msgs::msg::Image,             // Semantic mask
        zed_msgs::msg::OpticalFlowStamped    // Optical flow
    > ZedImageSyncPolicy;

    std::unique_ptr<image_transport::SubscriberFilter> rgb_sub_;
    std::unique_ptr<image_transport::SubscriberFilter> depth_sub_;
    std::unique_ptr<image_transport::SubscriberFilter> semantic_mask_sub_;
    std::unique_ptr<message_filters::Subscriber<zed_msgs::msg::OpticalFlowStamped>> optical_flow_sub_;
    std::unique_ptr<message_filters::Synchronizer<ZedImageSyncPolicy>> image_sync_;

    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<zed_msgs::msg::ObjectsStamped>::SharedPtr detected_objects_sub_;
    rclcpp::Subscription<zed_msgs::msg::ObjectsStamped>::SharedPtr detected_skeletons_sub_;

    ThreadsafeImuBuffer imu_buffer_; // From DynOSAM core

    mutable std::mutex camera_info_mutex_;
    CameraParams::Optional camera_params_;
    std::atomic<bool> camera_info_received_ = false;
    std::condition_variable camera_info_cond_;

    FrameId frame_id_counter_ = 0;
    Timestamp last_image_timestamp_ns_ = 0; // For querying IMU data, in nanoseconds
    
    // Storage for detected objects and skeletons
    mutable std::mutex objects_mutex_;
    zed_msgs::msg::ObjectsStamped::SharedPtr latest_detected_objects_;
    zed_msgs::msg::ObjectsStamped::SharedPtr latest_detected_skeletons_;
    
    std::atomic<bool> shutdown_ = false;
};

}