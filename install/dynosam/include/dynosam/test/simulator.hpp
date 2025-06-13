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

#pragma once

#include <gtsam/geometry/Pose3.h>

#include "dynosam/common/Types.hpp"
#include "dynosam/frontend/RGBDInstance-Definitions.hpp"
#include "dynosam/utils/GtsamUtils.hpp"

namespace dyno_testing {

using namespace dyno;

class ScenarioBodyBase {
 public:
  DYNO_POINTER_TYPEDEFS(ScenarioBodyBase)

  virtual ~ScenarioBodyBase() {}

  virtual gtsam::Pose3 pose(FrameId frame_id) const = 0;  ///< pose at time t
  virtual gtsam::Pose3 motionWorld(
      FrameId frame_id) const = 0;  ///< motion in world frame from t-1 to t
  virtual gtsam::Pose3 motionBody(FrameId frame_id)
      const = 0;  ///< motion local frame from t-1 to t, in ^{t-1}X_{t-1}
  virtual gtsam::Pose3 motionWorldFromInitial(
      FrameId frame_id) const = 0;  ///< motion in world frame from 0 to t

  gtsam::Rot3 rotation(FrameId frame_id) const {
    return this->pose(frame_id).rotation();
  }
  gtsam::Vector3 translation(FrameId frame_id) const {
    return this->pose(frame_id).translation();
  }
};

class ScenarioBodyVisitor : public ScenarioBodyBase {
 public:
  DYNO_POINTER_TYPEDEFS(ScenarioBodyVisitor)
  virtual ~ScenarioBodyVisitor() {}

  virtual gtsam::Pose3 pose(FrameId frame_id) const = 0;  ///< pose at time t
  virtual gtsam::Pose3 motionWorld(
      FrameId frame_id) const = 0;  ///< motion in world frame from t-1 to t

  ///< motion local frame from t-1 to t, in ^{t-1}X_{t-1}
  virtual gtsam::Pose3 motionBody(FrameId frame_id) const override {
    // from t-1 to t
    const gtsam::Pose3 motion_k = motionWorld(frame_id);
    const gtsam::Pose3 pose_k = pose(frame_id);
    // TODO: check
    return pose_k.inverse() * motion_k * pose_k.inverse();
  }
  virtual gtsam::Pose3 motionWorldFromInitial(
      FrameId frame_id) const = 0;  ///< motion in world frame from 0 to t
};

class ScenarioBody : public ScenarioBodyBase {
 public:
  DYNO_POINTER_TYPEDEFS(ScenarioBody)

  ScenarioBody(ScenarioBodyVisitor::UniquePtr body_visitor)
      : body_visitor_(std::move(body_visitor)) {}

  gtsam::Pose3 pose(FrameId frame_id) const override {
    return body_visitor_->pose(frame_id);
  }
  gtsam::Pose3 motionWorld(FrameId frame_id) const override {
    return body_visitor_->motionWorld(frame_id);
  }
  gtsam::Pose3 motionBody(FrameId frame_id) const override {
    return body_visitor_->motionBody(frame_id);
  }
  gtsam::Pose3 motionWorldFromInitial(FrameId frame_id) const override {
    return body_visitor_->motionWorldFromInitial(frame_id);
  }

 protected:
  ScenarioBodyVisitor::UniquePtr body_visitor_;
};

using TrackedPoint = std::pair<TrackletId, gtsam::Point3>;
using TrackedPoints = std::vector<TrackedPoint>;

struct PointsGenerator {
  /**
   * @brief Static function to generate a unique tracklet for any generator. If
   * increment is true, the global tracklet id will be incremented
   *
   * @param increment
   * @return TrackletId
   */
  static TrackletId getTracklet(bool increment = true) {
    static TrackletId global_tracklet = 0;

    auto tracklet_id = global_tracklet;

    if (increment) global_tracklet++;
    return tracklet_id;
  }

  static TrackedPoint generateNewPoint(const gtsam::Point3& mean, double sigma,
                                       int32_t seed = 42) {
    gtsam::Point3 point = dyno::utils::perturbWithNoise(mean, sigma, seed);
    return std::make_pair(PointsGenerator::getTracklet(true), point);
  }
};

/**
 * @brief Base class that knows how to generate points given the
 * ScenarioBodyVisitor for an object
 *
 */
class ObjectPointGeneratorVisitor {
 public:
  DYNO_POINTER_TYPEDEFS(ObjectPointGeneratorVisitor)

  virtual ~ObjectPointGeneratorVisitor() = default;
  virtual TrackedPoints getPointsWorld(
      const ScenarioBodyVisitor::UniquePtr& body_visitor,
      FrameId frame_id) const = 0;
};

class StaticPointGeneratorVisitor {
 public:
  DYNO_POINTER_TYPEDEFS(StaticPointGeneratorVisitor)

  virtual ~StaticPointGeneratorVisitor() = default;
  virtual TrackedPoints getPointsWorld(FrameId frame_id) const = 0;
};

class ObjectBody : public ScenarioBody {
 public:
  DYNO_POINTER_TYPEDEFS(ObjectBody)

  // struct Params {
  //     double enters_scenario_ = 0.0;
  //     double leaves_scenario_ = std::numeric_limits<double>::max();
  // };

  ObjectBody(ScenarioBodyVisitor::UniquePtr body_visitor,
             ObjectPointGeneratorVisitor::UniquePtr points_visitor)
      : ScenarioBody(std::move(body_visitor)),
        points_visitor_(std::move(points_visitor)) {}

  virtual FrameId entersScenario() const { return 0; };
  virtual FrameId leavesScenario() const {
    return std::numeric_limits<FrameId>::max();
  };
  virtual TrackedPoints getPointsWorld(FrameId frame_id) const {
    return points_visitor_->getPointsWorld(body_visitor_, frame_id);
  };

 protected:
  ObjectPointGeneratorVisitor::UniquePtr points_visitor_;
};

// Motion and pose visotors
class ConstantMotionBodyVisitor : public ScenarioBodyVisitor {
 public:
  DYNO_POINTER_TYPEDEFS(ConstantMotionBodyVisitor)
  ConstantMotionBodyVisitor(const gtsam::Pose3& pose_0,
                            const gtsam::Pose3& motion)
      : pose_0_(pose_0), motion_(motion) {}

  virtual gtsam::Pose3 pose(FrameId frame_id) const override {
    // from Pose Changes From a Different Point of View
    return motionWorldFromInitial(frame_id) * pose_0_;
  }

  virtual gtsam::Pose3 motionWorld(FrameId) const override { return motion_; }

  // TODO: I have no idea if this is right for constant motion but whatevs...
  gtsam::Pose3 motionWorldFromInitial(FrameId frame_id) const {
    return gtsam::Pose3::Expmap(frame_id * gtsam::Pose3::Logmap(motion_));
  }

 private:
  const gtsam::Pose3 pose_0_;
  const gtsam::Pose3 motion_;
};

// Points generator visitor
class ConstantObjectPointsVisitor : public ObjectPointGeneratorVisitor {
 public:
  ConstantObjectPointsVisitor(size_t num_points) : num_points_(num_points) {}

  // TODO: this assumes that the points we get from the object are ALWAYS the
  // same and ALWAYS the same order
  //
  TrackedPoints getPointsWorld(
      const ScenarioBodyVisitor::UniquePtr& body_visitor,
      FrameId frame_id) const override {
    if (!is_init) {
      initalisePoints(body_visitor->pose(0));
    }

    TrackedPoints points_world_t;  // points in world frame at time t
    for (const auto& tracked_point : points_world_0_) {
      auto tracklet_id = tracked_point.first;
      auto point = tracked_point.second;
      points_world_t.push_back(std::make_pair(
          tracklet_id, body_visitor->motionWorldFromInitial(frame_id) * point));
    }

    return points_world_t;
  }

 private:
  void initalisePoints(const gtsam::Pose3& P0) const {
    std::mt19937 engine(42);
    std::uniform_real_distribution<double> normal(0.0, 1.0);

    for (size_t i = 0; i < num_points_; i++) {
      // generate around pose0 with a normal distrubution around the translation
      // component
      // gtsam::Point3 p(P0.x() + normal(engine), P0.y() + normal(engine),
      //           P0.z() + normal(engine));

      // points_world_0_.push_back(std::make_pair(PointsGenerator::getTracklet(true),
      // p));
      points_world_0_.push_back(
          PointsGenerator::generateNewPoint(P0.translation(), 1.0));
    }

    is_init = true;
  }

  const size_t num_points_;

  // mutable so can be changed in the initalised poitns function, which is
  // called once
  mutable TrackedPoints points_world_0_;  // points in the world frame at time 0
  mutable bool is_init{false};
};

// I think this only ever means that a point can be seen by a max o
class SimpleStaticPointsGenerator : public StaticPointGeneratorVisitor {
 public:
  SimpleStaticPointsGenerator(size_t num_points_per_frame, size_t overlap)
      : num_points_per_frame_(num_points_per_frame),
        overlap_(overlap),
        has_overlap_(overlap < num_points_per_frame) {}

  TrackedPoints getPointsWorld(FrameId frame_id) const override {
    // expect we always start at zero
    if (frame_id == 0) {
      generateNewPoints(num_points_per_frame_);
      return points_world_0_;
    } else {
      // must have at least this many points after the first (zeroth) frame
      CHECK_GE(points_world_0_.size(), num_points_per_frame_);

      CHECK(has_overlap_) << "not implemented";
      int diff = (int)num_points_per_frame_ - (int)overlap_;
      CHECK(diff > 0);
      generateNewPoints((size_t)diff);

      size_t start_i = frame_id * ((size_t)diff);
      CHECK_GT(start_i, 0);

      size_t end_i = start_i + num_points_per_frame_ - 1;
      CHECK_LT(end_i, points_world_0_.size());

      TrackedPoints points_in_window;
      for (size_t i = start_i; i <= end_i; i++) {
        points_in_window.push_back(points_world_0_.at(i));
      }

      CHECK_EQ(points_in_window.size(), num_points_per_frame_);
      return points_in_window;
    }
  }

 private:
  void generateNewPoints(size_t num_new) const {
    // points can be distributed over this distance
    constexpr double point_distance_sigma = 40;
    for (size_t i = 0; i < num_new; i++) {
      points_world_0_.push_back(PointsGenerator::generateNewPoint(
          gtsam::Point3(0, 0, 0), point_distance_sigma));
    }
  }

  const size_t num_points_per_frame_;
  const size_t overlap_;
  const bool has_overlap_;
  mutable TrackedPoints
      points_world_0_;  // all points in the world frame at time 0. This may be
                        // uppdated overtime within the getPointsWorld
};

class Scenario {
 public:
  Scenario(ScenarioBody::Ptr camera_body,
           StaticPointGeneratorVisitor::Ptr static_points_generator)
      : camera_body_(camera_body),
        static_points_generator_(static_points_generator) {}

  void addObjectBody(ObjectId object_id, ObjectBody::Ptr object_body) {
    CHECK_GT(object_id, background_label);
    object_bodies_.insert2(object_id, object_body);
  }

  gtsam::Pose3 cameraPose(FrameId frame_id) const {
    return camera_body_->pose(frame_id);
  }

  ObjectIds getObjectIds(FrameId frame_id) const {
    ObjectIds object_ids;
    for (const auto& [object_id, obj] : object_bodies_) {
      if (objectInScenario(object_id, frame_id))
        object_ids.push_back(object_id);
    }

    return object_ids;
  }

  bool objectInScenario(ObjectId object_id, FrameId frame_id) const {
    if (object_bodies_.exists(object_id)) {
      const auto& object = object_bodies_.at(object_id);

      return frame_id >= object->entersScenario() &&
             frame_id < object->leavesScenario();
    }
    return false;
  }

 protected:
  ScenarioBody::Ptr camera_body_;
  StaticPointGeneratorVisitor::Ptr static_points_generator_;
  gtsam::FastMap<ObjectId, ObjectBody::Ptr> object_bodies_;
};

class RGBDScenario : public Scenario {
 public:
  RGBDScenario(ScenarioBody::Ptr camera_body,
               StaticPointGeneratorVisitor::Ptr static_points_generator)
      : Scenario(camera_body, static_points_generator) {}

  RGBDInstanceOutputPacket::Ptr getOutput(FrameId frame_id) const {
    StatusLandmarkVector static_landmarks, dynamic_landmarks;
    StatusKeypointVector static_keypoint_measurements,
        dynamic_keypoint_measurements;

    MotionEstimateMap motions;
    const gtsam::Pose3 X_world = cameraPose(frame_id);

    // tracklets should be uniqyue but becuase we use the DynamicPointSymbol
    // they only need to be unique per frame
    for (const auto& [object_id, object] : object_bodies_) {
      if (objectInScenario(object_id, frame_id)) {
        const gtsam::Pose3 H_world_k = object->motionWorld(frame_id);
        TrackedPoints points_world = object->getPointsWorld(frame_id);

        motions.insert2(object_id,
                        dyno::ReferenceFrameValue<gtsam::Pose3>(
                            H_world_k, dyno::ReferenceFrame::GLOBAL));

        // convert to status vectors
        for (const TrackedPoint& tracked_p_world : points_world) {
          auto tracklet_id = tracked_p_world.first;
          auto p_world = tracked_p_world.second;
          const gtsam::Point3 p_camera = X_world.inverse() * p_world;

          // TODO: covariance
          MeasurementWithCovariance<Landmark> lmk_measurement(p_camera);
          auto landmark_status = dyno::LandmarkStatus::DynamicInLocal(
              lmk_measurement, frame_id, tracklet_id, object_id);

          dynamic_landmarks.push_back(landmark_status);

          // the keypoint sttatus should be unused in the RGBD case but
          // we need it to fill out the data structures
          // TODO: covariance?
          MeasurementWithCovariance<Keypoint> kp_measurement{dyno::Keypoint()};
          auto keypoint_status = dyno::KeypointStatus::DynamicInLocal(
              kp_measurement, frame_id, tracklet_id, object_id);
          dynamic_keypoint_measurements.push_back(keypoint_status);
        }
      }
    }

    // add static points
    const TrackedPoints static_points_world =
        static_points_generator_->getPointsWorld(frame_id);

    // convert to status vectors
    for (const TrackedPoint& tracked_p_world : static_points_world) {
      auto tracklet_id = tracked_p_world.first;
      auto p_world = tracked_p_world.second;
      const gtsam::Point3 p_camera = X_world.inverse() * p_world;

      // TODO: covariance
      MeasurementWithCovariance<Landmark> lmk_measurement(p_camera);
      auto landmark_status =
          dyno::LandmarkStatus::StaticInLocal(p_camera, frame_id, tracklet_id);
      static_landmarks.push_back(landmark_status);

      // the keypoint sttatus should be unused in the RGBD case but
      // we need it to fill out the data structures
      MeasurementWithCovariance<Keypoint> kp_measurement{dyno::Keypoint()};
      auto keypoint_status = dyno::KeypointStatus::StaticInLocal(
          kp_measurement, frame_id, tracklet_id);
      static_keypoint_measurements.push_back(keypoint_status);
    }

    return std::make_shared<RGBDInstanceOutputPacket>(
        static_keypoint_measurements, dynamic_keypoint_measurements,
        static_landmarks, dynamic_landmarks, X_world, frame_id, frame_id,
        motions);
  }
};

}  // namespace dyno_testing
