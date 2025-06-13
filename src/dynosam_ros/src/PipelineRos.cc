/*
 *   Copyright (c) 2023 ACFR-RPG, University of Sydney, Jesse Morris
 (jesse.morris@sydney.edu.au)
 *   All rights reserved.

 *   Permission is hereby granted, free of charge, to any person obtaining a
 copy
 *   of this software and associated documentation files (the "Software"), to
 deal
 *   in the Software without restriction, including without limitation the
 rights
 *   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *   copies of the Software, and to permit persons to whom the Software is
 *   furnished to do so, subject to the following conditions:

 *   The above copyright notice and this permission notice shall be included in
 all
 *   copies or substantial portions of the Software.

 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 FROM,
 *   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 THE
 *   SOFTWARE.
 */

#include "dynosam_ros/PipelineRos.hpp"

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <dynosam/dataprovider/DataProviderFactory.hpp>
#include <dynosam/dataprovider/DataProviderUtils.hpp>
#include <dynosam/pipeline/PipelineParams.hpp>
#include <dynosam/visualizer/OpenCVFrontendDisplay.hpp>

#include "dynosam_ros/Display-Definitions.hpp"
#include "dynosam_ros/OnlineDataProviderRos.hpp"
#include "dynosam_ros/ZEDOnlineDataProvider.hpp"
#include "dynosam_ros/RosUtils.hpp"
#include "dynosam_ros/displays/DisplaysImpl.hpp"
#include "rcl_interfaces/msg/parameter.hpp"
#include "rclcpp/parameter.hpp"

namespace dyno {

DynoNode::DynoNode(const std::string& node_name,
                   const rclcpp::NodeOptions& options)
    : Node(node_name, "dynosam", options) {
  RCLCPP_INFO_STREAM(this->get_logger(), "Starting DynoNode");
  auto params_path = getParamsPath();
  RCLCPP_INFO_STREAM(this->get_logger(),
                     "Loading Dyno VO params from: " << params_path);
  dyno_params_ = std::make_unique<DynoParams>(params_path);
}

ZEDOnlineDataProviderParams DynoNode::parseZEDParams() {
  ZEDOnlineDataProviderParams zed_params;
  
  zed_params.rgb_topic_name =
      ParameterConstructor(this, "zed.rgb_topic", zed_params.rgb_topic_name)
          .description("ZED RGB image topic name")
          .finish()
          .get<std::string>();
          
  zed_params.depth_topic_name =
      ParameterConstructor(this, "zed.depth_topic", zed_params.depth_topic_name)
          .description("ZED depth image topic name")
          .finish()
          .get<std::string>();
          
  zed_params.camera_info_topic_name =
      ParameterConstructor(this, "zed.camera_info_topic", zed_params.camera_info_topic_name)
          .description("ZED camera info topic name")
          .finish()
          .get<std::string>();
          
  zed_params.imu_topic_name =
      ParameterConstructor(this, "zed.imu_topic", zed_params.imu_topic_name)
          .description("ZED IMU topic name")
          .finish()
          .get<std::string>();
          
  zed_params.image_sync_queue_size =
      ParameterConstructor(this, "zed.image_sync_queue_size", zed_params.image_sync_queue_size)
          .description("Image synchronization queue size")
          .finish()
          .get<int>();
          
  zed_params.image_sync_slop_sec =
      ParameterConstructor(this, "zed.image_sync_slop_sec", zed_params.image_sync_slop_sec)
          .description("Image synchronization time tolerance (seconds)")
          .finish()
          .get<double>();
          
  zed_params.wait_for_camera_info =
      ParameterConstructor(this, "zed.wait_for_camera_info", zed_params.wait_for_camera_info)
          .description("Wait for camera info message on startup")
          .finish()
          .get<bool>();
          
  int timeout_ms =
      ParameterConstructor(this, "zed.camera_info_timeout_ms", static_cast<int>(zed_params.camera_info_timeout_ms.count()))
          .description("Timeout for waiting for camera info (ms, -1 for infinite)")
          .finish()
          .get<int>();
  zed_params.camera_info_timeout_ms = std::chrono::milliseconds(timeout_ms);
  
  zed_params.imu_buffer_size =
      ParameterConstructor(this, "zed.imu_buffer_size", static_cast<int>(zed_params.imu_buffer_size))
          .description("IMU buffer size")
          .finish()
          .get<int>();
          
  zed_params.output_rgb =
      ParameterConstructor(this, "zed.output_rgb", zed_params.output_rgb)
          .description("Output RGB (true) or grayscale (false)")
          .finish()
          .get<bool>();
          
  zed_params.enable_imu_processing =
      ParameterConstructor(this, "zed.enable_imu_processing", zed_params.enable_imu_processing)
          .description("Enable IMU processing")
          .finish()
          .get<bool>();
          
  return zed_params;
}

dyno::DataProvider::Ptr DynoNode::createDataProvider() {
  const bool online =
      ParameterConstructor(this, "online", false)
          .description("If the online DataProvider should be used")
          .finish()
          .get<bool>();

  if (online) {
    // Check if ZED provider should be used
    const std::string online_provider_type =
        ParameterConstructor(this, "online_provider_type", std::string("generic"))
            .description("Type of online data provider: 'generic' or 'zed'")
            .finish()
            .get<std::string>();
    
    if (online_provider_type == "zed") {
      RCLCPP_INFO_STREAM(
          this->get_logger(),
          "ZED Online DataProvider selected. Waiting for ZED ROS topics...");
      
      auto zed_params = parseZEDParams();
      return std::make_shared<ZEDOnlineDataProvider>(
          this->create_sub_node("dataprovider"), zed_params);
    } else {
      RCLCPP_INFO_STREAM(
          this->get_logger(),
          "Generic Online DataProvider selected. Waiting for ROS topics...");
      
      OnlineDataProviderRosParams online_params;
      online_params.wait_for_camera_params =
          ParameterConstructor(this, "wait_for_camera_params",
                               online_params.wait_for_camera_params)
              .description(
                  "If the online DataProvider should wait for the camera params "
                  "on a ROS topic!")
              .finish()
              .get<bool>();
      online_params.camera_params_timeout =
          ParameterConstructor(this, "camera_params_timeout",
                               online_params.camera_params_timeout)
              .description(
                  "When waiting for camera params, how long the online "
                  "DataProvider should wait before time out (ms)")
              .finish()
              .get<int>();
      return std::make_shared<OnlineDataProviderRos>(
          this->create_sub_node("dataprovider"), online_params);
    }
  }

  auto params_path = getParamsPath();
  auto dataset_path = getDatasetPath();
  auto dyno_params = getDynoParams();

  RCLCPP_INFO_STREAM(this->get_logger(),
                     "Loading dataset from: " << dataset_path);

  dyno::DataProvider::Ptr data_loader = dyno::DataProviderFactory::Create(
      dataset_path, params_path,
      static_cast<dyno::DatasetType>(dyno_params.dataProviderType()));
  RCLCPP_INFO_STREAM(this->get_logger(), "Constructed data loader");
  return data_loader;
}

std::string DynoNode::searchForPathWithParams(const std::string& param_name,
                                              const std::string& default_path,
                                              const std::string& description) {
  // check if we've alrady declared this param
  // use non-default version so that ParameterConstructor throws exception if no
  // parameter is provided on the param server
  const std::string path = ParameterConstructor(this, param_name)
                               .description(description)
                               .finish()
                               .get<std::string>();
  throwExceptionIfPathInvalid(path);
  return path;
}

DynoPipelineManagerRos::DynoPipelineManagerRos(
    const rclcpp::NodeOptions& options)
    : DynoNode("dynosam", options) {}

void DynoPipelineManagerRos::initalisePipeline() {
  RCLCPP_INFO_STREAM(this->get_logger(), "Starting DynoPipelineManagerRos");

  auto params = getDynoParams();

  // setup display params
  DisplayParams display_params;
  display_params.camera_frame_id =
      ParameterConstructor(this, "camera_frame_id",
                           display_params.camera_frame_id)
          .description(
              "ROS frame id for the camera (ie. the measured odometry)")
          .finish()
          .get<std::string>();
  display_params.world_frame_id =
      ParameterConstructor(this, "world_frame_id",
                           display_params.world_frame_id)
          .description("ROS frame id for the static workd frame (ie. odometry)")
          .finish()
          .get<std::string>();

  auto frontend_display = std::make_shared<dyno::FrontendDisplayRos>(
      display_params, this->create_sub_node("frontend"));
  auto backend_display = std::make_shared<dyno::BackendDisplayRos>(
      display_params, this->create_sub_node("backend"));

  auto data_loader = createDataProvider();
  pipeline_ = std::make_unique<DynoPipelineManager>(
      params, data_loader, frontend_display, backend_display);
}

}  // namespace dyno
