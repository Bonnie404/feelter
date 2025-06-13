/*
 *   Copyright (c) 2024 ACFR-RPG, University of Sydney, Jesse Morris
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

#include "dynosam/frontend/RGBDInstance-Definitions.hpp"

#include <glog/logging.h>

#include "dynosam/common/Types.hpp"
#include "dynosam/frontend/vision/VisionTools.hpp"
#include "dynosam/logger/Logger.hpp"

namespace dyno {

GenericTrackedStatusVector<LandmarkKeypointStatus>
collectLandmarkKeypointMeasurementsHelper(
    const StatusLandmarkVector& landmarks,
    const StatusKeypointVector& keypoints) {
  CHECK_EQ(landmarks.size(), keypoints.size());

  GenericTrackedStatusVector<LandmarkKeypointStatus> collection;

  for (size_t i = 0; i < landmarks.size(); i++) {
    const auto& lmk_status = landmarks.at(i);
    const auto& kp_status = keypoints.at(i);

    CHECK_EQ(lmk_status.trackletId(), kp_status.trackletId());
    CHECK_EQ(lmk_status.objectId(), kp_status.objectId());
    CHECK_EQ(lmk_status.frameId(), kp_status.frameId());
    // expect visual measurements being sent to the back-end to be local
    CHECK_EQ(lmk_status.referenceFrame(), ReferenceFrame::LOCAL);

    collection.push_back(LandmarkKeypointStatus(
        LandmarkKeypoint(lmk_status.value(), kp_status.value()),
        lmk_status.frameId(), lmk_status.trackletId(), lmk_status.objectId(),
        lmk_status.referenceFrame()));
  }

  return collection;
}

GenericTrackedStatusVector<LandmarkKeypointStatus>
RGBDInstanceOutputPacket::collectStaticLandmarkKeypointMeasurements() const {
  return collectLandmarkKeypointMeasurementsHelper(
      static_landmarks_, static_keypoint_measurements_);
}
GenericTrackedStatusVector<LandmarkKeypointStatus>
RGBDInstanceOutputPacket::collectDynamicLandmarkKeypointMeasurements() const {
  return collectLandmarkKeypointMeasurementsHelper(
      dynamic_landmarks_, dynamic_keypoint_measurements_);
}

RGBDFrontendLogger::RGBDFrontendLogger()
    : EstimationModuleLogger("frontend"),
      tracking_length_hist_file_name_(
          getOutputFilePath("tracklet_length_hist.json")) {}

void RGBDFrontendLogger::logTrackingLengthHistogram(const Frame::Ptr frame) {
  gtsam::FastMap<ObjectId, Histogram> histograms =
      vision_tools::makeTrackletLengthHistorgram(frame);
  // collect histograms per object and then nest them per frame
  // must cast keys (object id, frame id) to string to get the json library to
  // properly construct nested maps
  json per_object_hist;
  for (const auto& [object_id, hist] : histograms) {
    per_object_hist[std::to_string(object_id)] = hist;
  }
  tracklet_length_json_[std::to_string(frame->getFrameId())] = per_object_hist;
}

RGBDFrontendLogger::~RGBDFrontendLogger() {
  JsonConverter::WriteOutJson(tracklet_length_json_,
                              tracking_length_hist_file_name_);
}

}  // namespace dyno
