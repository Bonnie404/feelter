/*
 *   Copyright (c) 2024 ACFR-RPG, University of Sydney, Jesse Morris (jesse.morris@sydney.edu.au)
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

#include "dynosam/backend/Formulation.hpp"
#include "dynosam/backend/Accessor.hpp"
#include "dynosam/common/Map.hpp"

#include "dynosam/backend/rgbd/WorldPoseEstimator.hpp"

namespace dyno {


class WorldMotionAccessor : public WorldPoseAccessor {
    public:
        WorldMotionAccessor(const gtsam::Values* theta, Map3d2d::Ptr map) : WorldPoseAccessor(theta, map) {}

        StateQuery<gtsam::Pose3> getObjectMotion(FrameId frame_id, ObjectId object_id) const override;
        StateQuery<gtsam::Pose3> getObjectPose(FrameId frame_id, ObjectId object_id) const override;

        inline ObjectPoseMap getObjectPoses() const override { return object_pose_cache_; }
        EstimateMap<ObjectId, gtsam::Pose3> getObjectPoses(FrameId frame_id) const override;

        //this will update the object_pose_cache_ so call it every frame?
        void postUpdateCallback(const BackendMetaData&) override;

    private:
        ObjectPoseMap object_pose_cache_; //! Updated every time set/update theta is called via the postUpdateCallback
};

class WorldMotionFormulation : public WorldPoseFormulation {
public:
    using Base = WorldPoseFormulation;
    using Base::PointUpdateContextType;
    using Base::ObjectUpdateContextType;
    using Base::AccessorTypePointer;

    DYNO_POINTER_TYPEDEFS(WorldMotionFormulation)

    WorldMotionFormulation(const FormulationParams& params, typename Map::Ptr map, const NoiseModels& noise_models) : WorldPoseFormulation(params, map, noise_models) {}


    void dynamicPointUpdateCallback(const PointUpdateContextType& context, UpdateObservationResult& result, gtsam::Values& new_values,
        gtsam::NonlinearFactorGraph& new_factors) override;
    void objectUpdateContext(const ObjectUpdateContextType& context, UpdateObservationResult& result, gtsam::Values& new_values,
        gtsam::NonlinearFactorGraph& new_factors) override;

protected:
    AccessorTypePointer createAccessor(const gtsam::Values* values) const override {
        return std::make_shared<WorldMotionAccessor>(values, this->map());
    }

    std::string loggerPrefix() const override {
        return "rgbd_motion_world";
    }

};

} //dyno
