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

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <Eigen/Dense>
#include <png++/png.hpp>

#include "dynosam/common/Camera.hpp"
#include "dynosam/common/ImageContainer.hpp"
#include "dynosam/dataprovider/KittiDataProvider.hpp"
#include "dynosam/dataprovider/VirtualKittiDataProvider.hpp"
#include "dynosam/frontend/vision/FeatureTracker.hpp"
#include "dynosam/frontend/vision/Frame.hpp"
#include "dynosam/pipeline/PipelineManager.hpp"
#include "dynosam/pipeline/PipelineParams.hpp"
#include "dynosam/visualizer/ColourMap.hpp"
#include "dynosam/visualizer/OpenCVFrontendDisplay.hpp"

DEFINE_string(path_to_kitti, "/root/data/kitti", "Path to KITTI dataset");
// TODO: (jesse) many better ways to do this with ros - just for now
DEFINE_string(
    params_folder_path, "dynosam/params",
    "Path to the folder containing the yaml files with the VIO parameters.");

#include "dynosam/dataprovider/KittiDataProvider.hpp"
#include "dynosam/dataprovider/OMDDataProvider.hpp"
#include "dynosam/frontend/vision/VisionTools.hpp"

int main(int argc, char* argv[]) {
  using namespace dyno;
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);
  FLAGS_logtostderr = 1;
  FLAGS_colorlogtostderr = 1;
  FLAGS_log_prefix = 1;

  KittiDataLoader::Params params;
  // KittiDataLoader loader("/root/data/vdo_slam/kitti/kitti/0000/", params);
  // ClusterSlamDataLoader loader("/root/data/cluster_slam/CARLA-L1");
  OMDDataLoader loader(
      "");

  auto camera = std::make_shared<Camera>(*loader.getCameraParams());
  auto tracker = std::make_shared<FeatureTracker>(FrontendParams(), camera);

  loader.setCallback([&](dyno::FrameId frame_id, dyno::Timestamp timestamp,
                         cv::Mat rgb, cv::Mat optical_flow, cv::Mat depth,
                         cv::Mat motion, GroundTruthInputPacket) -> bool {
    // loader.setCallback([&](dyno::FrameId frame_id, dyno::Timestamp timestamp,
    // cv::Mat rgb, cv::Mat optical_flow, cv::Mat depth, cv::Mat motion,
    // gtsam::Pose3, GroundTruthInputPacket) -> bool {

    LOG(INFO) << frame_id << " " << timestamp;

    cv::Mat of_viz, motion_viz, depth_viz;
    of_viz = ImageType::OpticalFlow::toRGB(optical_flow);
    motion_viz = ImageType::MotionMask::toRGB(motion);
    depth_viz = ImageType::Depth::toRGB(depth);

    ImageContainer::Ptr container = ImageContainer::Create(
        timestamp, frame_id, ImageWrapper<ImageType::RGBMono>(rgb),
        ImageWrapper<ImageType::Depth>(depth),
        ImageWrapper<ImageType::OpticalFlow>(optical_flow),
        ImageWrapper<ImageType::MotionMask>(motion));

    // cv::Mat boarder_mask;
    // vision_tools::computeObjectMaskBoundaryMask(
    //     motion,
    //     boarder_mask,
    //     8
    // );

    // cv::Scalar red = dyno::Color::red();

    // const ObjectIds instance_labels = vision_tools::getObjectLabels(motion);
    // for(const auto object_id : instance_labels) {
    //     std::vector<std::vector<cv::Point>> detected_contours;
    //     vision_tools::findObjectBoundingBox(motion,
    //     object_id,detected_contours);

    //     cv::drawContours(boarder_mask, detected_contours, -1, red, 8);
    // }

    // cv::imshow("Mask with boarder", boarder_mask);

    // cv::imshow("RGB", rgb);
    // cv::imshow("OF", of_viz);
    cv::imshow("Motion", motion_viz);
    // cv::waitKey(1);
    // cv::imshow("Depth", depth_viz);

    auto frame = tracker->track(frame_id, timestamp, *container);
    Frame::Ptr previous_frame = tracker->getPreviousFrame();

    // // motion_viz =
    // ImageType::MotionMask::toRGB(frame->image_container_.get<ImageType::MotionMask>());
    // // // cv::imshow("Motion", motion_viz);

    cv::Mat tracking;
    if (previous_frame) {
      tracking = tracker->computeImageTracks(*previous_frame, *frame, false);
    }
    if (!tracking.empty()) cv::imshow("Tracking", tracking);

    LOG(INFO) << to_string(tracker->getTrackerInfo());
    const std::string path = "/root/results/misc/";
    if ((char)cv::waitKey(0) == 's') {
      LOG(INFO) << "Saving...";
      // cv::imwrite(path + "omd_su4_rgb.png", rgb);
      // cv::imwrite(path + "omd_su4_of.png", of_viz);
      // cv::imwrite(path + "omd_su4_motion.png", motion_viz);
      // cv::imwrite(path + "omd_su4_depth.png", depth_viz);
      cv::imwrite(path + "omd_tracking.png", tracking);
    }
    cv::waitKey(1);

    return true;
  });

  while (loader.spin()) {
  }
}

// #include "dynosam/dataprovider/ProjectAriaDataProvider.hpp"
// #include "dynosam/frontend/vision/VisionTools.hpp"

// int main(int argc, char* argv[]) {

//     using namespace dyno;
//     google::ParseCommandLineFlags(&argc, &argv, true);
//     google::InitGoogleLogging(argv[0]);
//     FLAGS_logtostderr = 1;
//     FLAGS_colorlogtostderr = 1;
//     FLAGS_log_prefix = 1;

//     // ClusterSlamDataLoader loader("/root/data/cluster_slam/CARLA-S1");
//     ProjectARIADataLoader loader("/root/data/zed/acfr_3_moving_medium/");

//     loader.setCallback([&](dyno::FrameId frame_id, dyno::Timestamp timestamp,
//     cv::Mat rgb, cv::Mat optical_flow, cv::Mat depth, cv::Mat motion) -> bool
//     {

//         LOG(INFO) << frame_id << " " << timestamp;

//         cv::imshow("RGB", rgb);
//         cv::imshow("OF", ImageType::OpticalFlow::toRGB(optical_flow));
//         cv::imshow("Motion", ImageType::MotionMask::toRGB(motion));
//         cv::imshow("Depth", ImageType::Depth::toRGB(depth));

//         cv::Mat shrunk_mask;
//         vision_tools::shrinkMask(motion, shrunk_mask, 20);
//         cv::imshow("Shrunk Motion",
//         ImageType::MotionMask::toRGB(shrunk_mask));

//         cv::waitKey(1);
//         return true;
//     });

//     while(loader.spin()) {}

// }
