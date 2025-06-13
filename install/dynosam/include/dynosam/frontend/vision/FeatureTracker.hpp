/*
 *   Copyright (c) 2023 ACFR-RPG, University of Sydney, Jesse Morris (jesse.morris@sydney.edu.au)
 *   All rights reserved.

 *   Permission is hereby granted, free of charge, to any person obtaining a copy
 *   of this software and associated documentation files (the "Software"), to deal
 *   in the Software without restriction, including without limitation the rights
 *   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *   copies of the Software, and to permit persons to whom the Software is
 *   furnished to do so, subject to the following conditions:

 *   The above copyright notice and this permission notice shall be included in all
 *   copies or substantial portions of the Software.

 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *   SOFTWARE.
 */

#pragma once

#include "dynosam/frontend/vision/FeatureTrackerBase.hpp"
#include "dynosam/frontend/vision/StaticFeatureTracker.hpp"
#include "dynosam/common/Camera.hpp"
#include "dynosam/frontend/vision/Frame.hpp"
#include "dynosam/frontend/vision/ORBextractor.hpp"
#include "dynosam/frontend/vision/Feature.hpp"
#include "dynosam/frontend/FrontendParams.hpp"
#include "dynosam/visualizer/Visualizer-Definitions.hpp"
#include "dynosam/frontend/vision/OccupancyGrid2D.hpp"

#include <opencv4/opencv2/opencv.hpp>

namespace dyno {

/**
 * @brief Feature detector that combines sparse static feature detection and tracking with dense feature detection and tracking on dynamic objects.
 *
 */
class FeatureTracker : public FeatureTrackerBase
{
public:
    DYNO_POINTER_TYPEDEFS(FeatureTracker)

    //and camera?
    //does no processing with any depth
    //if depth is a problem should be handled aftererds and separately
    FeatureTracker(const FrontendParams& params, Camera::Ptr camera, ImageDisplayQueue* display_queue = nullptr);
    virtual ~FeatureTracker() {}

    //note: MOTION MASK!!
    // Frame::Ptr track(FrameId frame_id, Timestamp timestamp, const TrackingInputImages& tracking_images);
    Frame::Ptr track(FrameId frame_id, Timestamp timestamp, const ImageContainer& image_container);

    /**
     * @brief Get the previous frame.
     *
     * Will be null on the first call of track
     *
     * @return Frame::Ptr
     */
    inline Frame::Ptr getPreviousFrame() { return previous_tracked_frame_; }
    inline const Frame::ConstPtr getPreviousFrame() const { return previous_tracked_frame_; }

    /**
     * @brief Get the most recent frame that has been tracked.
     * After a call to track, this will be the frame that is returned and will track features between getPreviousFrame() to this frame
     *
     * @return Frame::Ptr
     */
    inline Frame::Ptr getCurrentFrame() { return previous_frame_; }

    inline const FeatureTrackerInfo& getTrackerInfo() { return info_; }


protected:
    //detection mask is additional mask It must be a 8-bit integer matrix with non-zero values in the region of interest, indicating what featues to not track
    void trackDynamic(FrameId frame_id, const ImageContainer& image_container, FeatureContainer& dynamic_features, const cv::Mat& detection_mask = cv::Mat());

    void propogateMask(ImageContainer& image_container);

private:
    void computeImageBounds(const cv::Size& size, int& min_x, int& max_x, int& min_y, int& max_y) const;

    // Feature::Ptr constructStaticFeature(const ImageContainer& image_container, const Keypoint& kp, size_t age, TrackletId tracklet_id,
    //                                   FrameId frame_id) const;


private:
    const FrontendParams frontend_params_;
    Frame::Ptr previous_frame_{ nullptr }; //! The frame that will be used as the previous frame next time track is called. After track, this is actually the frame that track() returns
    Frame::Ptr previous_tracked_frame_{nullptr}; //! The frame that has just beed used to track on a new frame is created.
    // ORBextractor::UniquePtr feature_detector_{nullptr};
    StaticFeatureTracker::UniquePtr static_feature_tracker_;

    FeatureTrackerInfo info_;

    // OccupandyGrid2D static_grid_; //! Grid used to feature bin static features
    bool initial_computation_{ true };


};

}
