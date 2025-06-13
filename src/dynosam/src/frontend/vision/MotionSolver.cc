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

#include "dynosam/frontend/vision/MotionSolver.hpp"

#include <config_utilities/config_utilities.h>
#include <glog/logging.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>  //for now? //TODO: clean
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include <eigen3/Eigen/Dense>
#include <opencv4/opencv2/core.hpp>
#include <opencv4/opencv2/core/eigen.hpp>
#include <opengv/types.hpp>

#include "dynosam/backend/BackendDefinitions.hpp"
#include "dynosam/backend/FactorGraphTools.hpp"  //TODO: clean
#include "dynosam/common/Flags.hpp"
#include "dynosam/common/Types.hpp"
#include "dynosam/factors/LandmarkMotionTernaryFactor.hpp"
#include "dynosam/factors/Pose3FlowProjectionFactor.h"
#include "dynosam/frontend/vision/VisionTools.hpp"
#include "dynosam/utils/Accumulator.hpp"
#include "dynosam/utils/GtsamUtils.hpp"
#include "dynosam/utils/Numerical.hpp"
#include "dynosam/utils/TimingStats.hpp"

namespace dyno {

void declare_config(OpticalFlowAndPoseOptimizer::Params& config) {
  using namespace config;

  name("OpticalFlowAndPoseOptimizerParams");
  field(config.flow_sigma, "flow_sigma");
  field(config.flow_prior_sigma, "flow_prior_sigma");
  field(config.k_huber, "k_huber");
  field(config.outlier_reject, "outlier_reject");
  field(config.flow_is_future, "flow_is_future");
}

void declare_config(MotionOnlyRefinementOptimizer::Params& config) {
  using namespace config;

  name("MotionOnlyRefinementOptimizerParams");
  field(config.landmark_motion_sigma, "landmark_motion_sigma");
  field(config.projection_sigma, "projection_sigma");
  field(config.k_huber, "k_huber");
  field(config.outlier_reject, "outlier_reject");
}

void declare_config(EgoMotionSolver::Params& config) {
  using namespace config;

  name("EgoMotionSolver::Params");
  field(config.ransac_randomize, "ransac_randomize");
  field(config.ransac_use_2point_mono, "ransac_use_2point_mono");
  field(config.optimize_2d2d_pose_from_inliers,
        "optimize_2d2d_pose_from_inliers");
  field(config.ransac_threshold_pnp, "ransac_threshold_pnp");
  field(config.optimize_3d2d_pose_from_inliers,
        "optimize_3d2d_pose_from_inliers");
  field(config.ransac_threshold_stereo, "ransac_threshold_stereo");
  field(config.optimize_3d3d_pose_from_inliers,
        "optimize_3d3d_pose_from_inliers");
  field(config.ransac_iterations, "ransac_iterations");
  field(config.ransac_probability, "ransac_probability");
}
void declare_config(ObjectMotionSovler::Params& config) {
  using namespace config;
  name("ObjectMotionSovler::Params");

  base<EgoMotionSolver::Params>(config);
  field(config.refine_motion_with_joint_of, "refine_motion_with_joint_of");
  field(config.refine_motion_with_3d, "refine_motion_with_3d");
  field(config.joint_of_params, "joint_optical_flow");
  field(config.object_motion_refinement_params, "object_motion_3d_refinement");
}

EgoMotionSolver::EgoMotionSolver(const Params& params,
                                 const CameraParams& camera_params)
    : params_(params), camera_params_(camera_params) {}

Pose3SolverResult EgoMotionSolver::geometricOutlierRejection2d2d(
    Frame::Ptr frame_k_1, Frame::Ptr frame_k,
    std::optional<gtsam::Rot3> R_curr_ref) {
  // get correspondences
  RelativePoseCorrespondences correspondences;
  // this does not create proper bearing vectors (at leas tnot for 3d-2d pnp
  // solve) bearing vectors are also not undistorted atm!!
  {
    utils::TimingStatsCollector track_dynamic_timer(
        "mono_frame_correspondences");
    frame_k->getCorrespondences(correspondences, *frame_k_1,
                                KeyPointType::STATIC,
                                frame_k->imageKeypointCorrespondance());
  }

  Pose3SolverResult result;

  const size_t& n_matches = correspondences.size();

  if (n_matches < 5u) {
    result.status = TrackingStatus::FEW_MATCHES;
    return result;
  }

  gtsam::Matrix K = camera_params_.getCameraMatrixEigen();
  K = K.inverse();

  TrackletIds tracklets;
  // NOTE: currently without distortion! the correspondences should be made into
  // bearing vector elsewhere!
  BearingVectors ref_bearing_vectors, cur_bearing_vectors;
  for (size_t i = 0u; i < n_matches; i++) {
    const auto& corres = correspondences.at(i);
    const Keypoint& ref_kp = corres.ref_;
    const Keypoint& cur_kp = corres.cur_;

    gtsam::Vector3 ref_versor = (K * gtsam::Vector3(ref_kp(0), ref_kp(1), 1.0));
    gtsam::Vector3 cur_versor = (K * gtsam::Vector3(cur_kp(0), cur_kp(1), 1.0));

    ref_versor = ref_versor.normalized();
    cur_versor = cur_versor.normalized();

    ref_bearing_vectors.push_back(ref_versor);
    cur_bearing_vectors.push_back(cur_versor);

    tracklets.push_back(corres.tracklet_id_);
  }

  RelativePoseAdaptor adapter(ref_bearing_vectors, cur_bearing_vectors);

  const bool use_2point_mono = params_.ransac_use_2point_mono && R_curr_ref;
  if (use_2point_mono) {
    adapter.setR12((*R_curr_ref).matrix());
  }

  gtsam::Pose3 best_result;
  std::vector<int> ransac_inliers;
  bool success = false;
  if (use_2point_mono) {
    success = runRansac<RelativePoseProblemGivenRot>(
        std::make_shared<RelativePoseProblemGivenRot>(adapter,
                                                      params_.ransac_randomize),
        params_.ransac_threshold_mono, params_.ransac_iterations,
        params_.ransac_probability, params_.optimize_2d2d_pose_from_inliers,
        best_result, ransac_inliers);
  } else {
    success = runRansac<RelativePoseProblem>(
        std::make_shared<RelativePoseProblem>(
            adapter, RelativePoseProblem::NISTER, params_.ransac_randomize),
        params_.ransac_threshold_mono, params_.ransac_iterations,
        params_.ransac_probability, params_.optimize_2d2d_pose_from_inliers,
        best_result, ransac_inliers);
  }

  if (!success) {
    result.status = TrackingStatus::INVALID;
  } else {
    constructTrackletInliers(result.inliers, result.outliers, correspondences,
                             ransac_inliers, tracklets);
    // NOTE: 2-point always returns the identity rotation, hence we have to
    // substitute it:
    if (use_2point_mono) {
      CHECK(R_curr_ref->equals(best_result.rotation()));
    }
    result.status = TrackingStatus::VALID;
    result.best_result = best_result;
  }

  return result;
}

Pose3SolverResult EgoMotionSolver::geometricOutlierRejection3d2d(
    Frame::Ptr frame_k_1, Frame::Ptr frame_k,
    std::optional<gtsam::Rot3> R_curr_ref) {
  AbsolutePoseCorrespondences correspondences;
  // this does not create proper bearing vectors (at leas tnot for 3d-2d pnp
  // solve) bearing vectors are also not undistorted atm!!
  // TODO: change to use landmarkWorldProjectedBearingCorrespondance and then
  // change motion solver to take already projected bearing vectors
  {
    frame_k->getCorrespondences(correspondences, *frame_k_1,
                                KeyPointType::STATIC,
                                frame_k->landmarkWorldKeypointCorrespondance());
  }

  return geometricOutlierRejection3d2d(correspondences, R_curr_ref);
}

Pose3SolverResult EgoMotionSolver::geometricOutlierRejection3d2d(
    const AbsolutePoseCorrespondences& correspondences,
    std::optional<gtsam::Rot3> R_curr_ref) {
  utils::TimingStatsCollector timer("motion_solver.solve_3d2d");
  Pose3SolverResult result;
  const size_t& n_matches = correspondences.size();

  if (n_matches < 5u) {
    result.status = TrackingStatus::FEW_MATCHES;
    VLOG(5) << "3D2D tracking failed as there are to few matches" << n_matches;
    return result;
  }

  gtsam::Matrix K = camera_params_.getCameraMatrixEigen();
  K = K.inverse();

  TrackletIds tracklets, inliers, outliers;
  // NOTE: currently without distortion! the correspondences should be made into
  // bearing vector elsewhere!
  BearingVectors bearing_vectors;
  Landmarks points;
  for (size_t i = 0u; i < n_matches; i++) {
    const AbsolutePoseCorrespondence& corres = correspondences.at(i);
    const Keypoint& kp = corres.cur_;
    // make Bearing vector
    gtsam::Vector3 versor = (K * gtsam::Vector3(kp(0), kp(1), 1.0));
    versor = versor.normalized();
    bearing_vectors.push_back(versor);

    points.push_back(corres.ref_);
    tracklets.push_back(corres.tracklet_id_);
  }

  const double reprojection_error = params_.ransac_threshold_pnp;
  const double avg_focal_length =
      0.5 * static_cast<double>(camera_params_.fx() + camera_params_.fy());
  const double threshold =
      1.0 - std::cos(std::atan(std::sqrt(2.0) * reprojection_error /
                               avg_focal_length));
  // const double threshold = params_.ransac_threshold_pnp;

  AbsolutePoseAdaptor adapter(bearing_vectors, points);

  gtsam::Pose3 best_result;
  std::vector<int> ransac_inliers;

  bool success = runRansac<AbsolutePoseProblem>(
      std::make_shared<AbsolutePoseProblem>(adapter,
                                            AbsolutePoseProblem::KNEIP),
      threshold, params_.ransac_iterations, params_.ransac_probability,
      params_.optimize_3d2d_pose_from_inliers, best_result, ransac_inliers);

  if (success) {
    constructTrackletInliers(result.inliers, result.outliers, correspondences,
                             ransac_inliers, tracklets);

    if (result.inliers.size() < 5u) {
      result.status = TrackingStatus::FEW_MATCHES;
    } else {
      result.status = TrackingStatus::VALID;
      result.best_result = best_result;
    }

  } else {
    result.status = TrackingStatus::INVALID;
  }

  return result;
}

void OpticalFlowAndPoseOptimizer::updateFrameOutliersWithResult(
    const Result& result, Frame::Ptr frame_k_1, Frame::Ptr frame_k) const {
  // //original flow image that goes from k to k+1 (gross, im sorry!)
  // TODO: use flow_is_future param
  const cv::Mat& flow_image =
      frame_k->image_container_.get<ImageType::OpticalFlow>();
  const cv::Mat& motion_mask =
      frame_k->image_container_.get<ImageType::MotionMask>();

  auto camera = frame_k->camera_;
  const auto& refined_inliers = result.inliers;
  const auto& refined_flows = result.best_result.refined_flows;

  // outliers from the result. We will update this vector with new outliers
  auto refined_outliers = result.outliers;

  for (size_t i = 0; i < refined_inliers.size(); i++) {
    TrackletId tracklet_id = refined_inliers.at(i);
    gtsam::Point2 refined_flow = refined_flows.at(i);

    const Feature::Ptr feature_k_1 = frame_k_1->at(tracklet_id);
    Feature::Ptr feature_k = frame_k->at(tracklet_id);

    CHECK_EQ(feature_k->objectId(), result.best_result.object_id);

    const Keypoint kp_k_1 = feature_k_1->keypoint();
    Keypoint refined_keypoint = kp_k_1 + refined_flow;

    // check boundaries?
    if (!camera->isKeypointContained(refined_keypoint)) {
      refined_outliers.push_back(tracklet_id);
      continue;
    }

    feature_k->keypoint(refined_keypoint);
    ObjectId predicted_label =
        functional_keypoint::at<ObjectId>(refined_keypoint, motion_mask);
    if (predicted_label != result.best_result.object_id) {
      refined_outliers.push_back(tracklet_id);
      // TODO: other fields of the feature does not get updated? Inconsistencies
      // as measured flow, predicted kp etc are no longer correct!!?
      continue;
    }

    // we now have to update the prediced keypoint using the original flow!!
    // TODO: code copied from feature tracker
    const int x = functional_keypoint::u(refined_keypoint);
    const int y = functional_keypoint::v(refined_keypoint);
    double flow_xe = static_cast<double>(flow_image.at<cv::Vec2f>(y, x)[0]);
    double flow_ye = static_cast<double>(flow_image.at<cv::Vec2f>(y, x)[1]);
    // the measured flow after the origin has been updated
    OpticalFlow new_measured_flow(flow_xe, flow_ye);
    feature_k->measuredFlow(new_measured_flow);
    // TODO: check predicted flow is within image
    Keypoint predicted_kp = Feature::CalculatePredictedKeypoint(
        refined_keypoint, new_measured_flow);
    feature_k->predictedKeypoint(predicted_kp);
  }

  // update tracks
  for (const auto& outlier_tracklet : refined_outliers) {
    Feature::Ptr feature_k_1 = frame_k_1->at(outlier_tracklet);
    Feature::Ptr feature_k = frame_k->at(outlier_tracklet);

    CHECK(feature_k_1->usable());
    CHECK(feature_k->usable());

    feature_k->markOutlier();
    feature_k_1->markOutlier();
  }

  // refresh depth information for each frame
  CHECK(frame_k->updateDepths());
}

Pose3SolverResult EgoMotionSolver::geometricOutlierRejection3d3d(
    Frame::Ptr frame_k_1, Frame::Ptr frame_k,
    std::optional<gtsam::Rot3> R_curr_ref) {
  PointCloudCorrespondences correspondences;
  {
    utils::TimingStatsCollector("pc_correspondences");
    frame_k->getCorrespondences(
        correspondences, *frame_k_1, KeyPointType::STATIC,
        frame_k->landmarkWorldPointCloudCorrespondance());
  }

  return geometricOutlierRejection3d3d(correspondences, R_curr_ref);
}

Pose3SolverResult EgoMotionSolver::geometricOutlierRejection3d3d(
    const PointCloudCorrespondences& correspondences,
    std::optional<gtsam::Rot3> R_curr_ref) {
  const size_t& n_matches = correspondences.size();

  Pose3SolverResult result;
  if (n_matches < 5) {
    result.status = TrackingStatus::FEW_MATCHES;
    return result;
  }

  TrackletIds tracklets;
  BearingVectors ref_bearing_vectors, cur_bearing_vectors;

  for (size_t i = 0u; i < n_matches; i++) {
    const auto& corres = correspondences.at(i);
    const Landmark& ref_lmk = corres.ref_;
    const Landmark& cur_lmk = corres.cur_;
    ref_bearing_vectors.push_back(ref_lmk);
    cur_bearing_vectors.push_back(cur_lmk);

    tracklets.push_back(corres.tracklet_id_);
  }

  //! Setup adapter.
  Adapter3d3d adapter(ref_bearing_vectors, cur_bearing_vectors);

  if (R_curr_ref) {
    adapter.setR12((*R_curr_ref).matrix());
  }

  gtsam::Pose3 best_result;
  std::vector<int> ransac_inliers;

  bool success = runRansac<Problem3d3d>(
      std::make_shared<Problem3d3d>(adapter, params_.ransac_randomize),
      params_.ransac_threshold_stereo, params_.ransac_iterations,
      params_.ransac_probability, params_.optimize_3d3d_pose_from_inliers,
      best_result, ransac_inliers);

  if (success) {
    constructTrackletInliers(result.inliers, result.outliers, correspondences,
                             ransac_inliers, tracklets);

    result.status = TrackingStatus::VALID;
    result.best_result = best_result;
  } else {
    result.status = TrackingStatus::INVALID;
  }

  return result;
}

ObjectMotionSovler::ObjectMotionSovler(const Params& params,
                                       const CameraParams& camera_params)
    : EgoMotionSolver(static_cast<const EgoMotionSolver::Params&>(params),
                      camera_params) {}

Pose3SolverResult ObjectMotionSovler::geometricOutlierRejection3d2d(
    Frame::Ptr frame_k_1, Frame::Ptr frame_k, const gtsam::Pose3& T_world_k,
    ObjectId object_id) {
  utils::TimingStatsCollector timer("motion_solver.object_solve3d2d");
  AbsolutePoseCorrespondences dynamic_correspondences;
  // get the corresponding feature pairs
  bool corr_result = frame_k->getDynamicCorrespondences(
      dynamic_correspondences, *frame_k_1, object_id,
      frame_k->landmarkWorldKeypointCorrespondance());

  const size_t& n_matches = dynamic_correspondences.size();

  TrackletIds all_tracklets;
  std::transform(dynamic_correspondences.begin(), dynamic_correspondences.end(),
                 std::back_inserter(all_tracklets),
                 [](const AbsolutePoseCorrespondence& corres) {
                   return corres.tracklet_id_;
                 });
  CHECK_EQ(all_tracklets.size(), n_matches);

  Pose3SolverResult geometric_result =
      EgoMotionSolver::geometricOutlierRejection3d2d(dynamic_correspondences);
  Pose3SolverResult result = geometric_result;

  // if(geometric_result.status == TrackingStatus::VALID &&
  // motion_model_result.status == TrackingStatus::VALID) {
  //     if(geometric_result.inliers.size() >=
  //     motion_model_result.inliers.size()) {
  //         VLOG(10) << "Geometric model used (inliers " <<
  //         geometric_result.inliers.size() << " vs " <<
  //         motion_model_result.inliers.size(); result = geometric_result;
  //     }
  //     else {
  //         VLOG(10) << "Motion motion model used (inliers " <<
  //         motion_model_result.inliers.size() << " vs " <<
  //         geometric_result.inliers.size(); result = motion_model_result;
  //     }
  // }
  // //if no motion model, fall back on gemoetric result
  // else {
  //     result = geometric_result;
  // }

  if (result.status == TrackingStatus::VALID) {
    TrackletIds refined_inlier_tracklets = result.inliers;

    {
      // debug only (just checking that the inlier/outliers we get from the
      // geometric rejection match the original one)
      TrackletIds extracted_all_tracklets = refined_inlier_tracklets;
      extracted_all_tracklets.insert(extracted_all_tracklets.end(),
                                     result.outliers.begin(),
                                     result.outliers.end());
      CHECK_EQ(all_tracklets.size(), extracted_all_tracklets.size());
    }

    gtsam::Pose3 G_w = result.best_result.inverse();
    if (object_motion_params.refine_motion_with_joint_of) {
      OpticalFlowAndPoseOptimizer flow_optimizer(
          object_motion_params.joint_of_params);
      // Use the original result as the input to the refine joint optical flow
      // function the result.best_result variable is actually equivalent to
      // ^wG^{-1} and we want to solve something in the form e(T, flow) =
      // [u,v]_{k-1} + {k-1}_flow_k - pi(T^{-1}^wm_{k-1}) so T must take the
      // point from k-1 in the world frame to the local frame at k-1 ^wG^{-1} =
      //^wX_k \: {k-1}^wH_k (which takes does this) but the error term uses the
      // inverse of T hence we must parse in the inverse of G
      auto flow_opt_result = flow_optimizer.optimizeAndUpdate<CalibrationType>(
          frame_k_1, frame_k, refined_inlier_tracklets, result.best_result);
      // still need to take the inverse as we get the inverse of G out
      G_w = flow_opt_result.best_result.refined_pose.inverse();
      // inliers should be a subset of the original refined inlier tracks
      refined_inlier_tracklets = flow_opt_result.inliers;
    }
    // still need to take the inverse as we get the inverse of G out
    gtsam::Pose3 H_w = T_world_k * G_w;

    // if(params_.refine_object_motion_esimate) {
    if (object_motion_params.refine_motion_with_3d) {
      MotionOnlyRefinementOptimizer motion_refinement_graph(
          object_motion_params.object_motion_refinement_params);
      auto motion_refinement_result =
          motion_refinement_graph.optimizeAndUpdate<CalibrationType>(
              frame_k_1, frame_k, refined_inlier_tracklets, object_id, H_w);

      // should be further subset
      refined_inlier_tracklets = motion_refinement_result.inliers;
      H_w = motion_refinement_result.best_result;
    }
    // a lot of weird places where we mark things as inliers,take results of
    // some functions into others etc.. and is very confusion!
    // TODO: clean up!!!
    //  result = result_copy;
    result.best_result = H_w;
    result.inliers = refined_inlier_tracklets;
    determineOutlierIds(result.inliers, all_tracklets, result.outliers);
  }

  // if not valid, return motion result as is
  return result;
}

Pose3SolverResult ObjectMotionSovler::geometricOutlierRejection3d3d(
    Frame::Ptr frame_k_1, Frame::Ptr frame_k, const gtsam::Pose3& T_world_k,
    ObjectId object_id) {
  PointCloudCorrespondences dynamic_correspondences;
  // get the corresponding feature pairs
  bool corr_result = frame_k->getDynamicCorrespondences(
      dynamic_correspondences, *frame_k_1, object_id,
      frame_k->landmarkWorldPointCloudCorrespondance());

  const size_t& n_matches = dynamic_correspondences.size();

  Pose3SolverResult result;
  result =
      EgoMotionSolver::geometricOutlierRejection3d3d(dynamic_correspondences);

  if (result.status == TrackingStatus::VALID) {
    const gtsam::Pose3 G_w = result.best_result.inverse();
    const gtsam::Pose3 H_w = T_world_k * G_w;
    result.best_result = H_w;
  }

  // if not valid, return motion result as is
  return result;
}

// TODO: dont actually need all these variables
Pose3SolverResult ObjectMotionSovler::motionModelOutlierRejection3d2d(
    const AbsolutePoseCorrespondences& dynamic_correspondences,
    Frame::Ptr frame_k_1, Frame::Ptr frame_k, const gtsam::Pose3& T_world_k,
    ObjectId object_id) {
  Pose3SolverResult result;

  // get object motions in previous frame (so k-2 to k-1)
  const MotionEstimateMap& motion_estiamtes_k_1 = frame_k_1->motion_estimates_;
  if (!motion_estiamtes_k_1.exists(object_id)) {
    result.status = TrackingStatus::INVALID;
    return result;
  }

  // previous motion model: k-2 to k-1 in w
  const gtsam::Pose3 motion_model = motion_estiamtes_k_1.at(object_id);

  using Calibration = Camera::CalibrationType;
  const auto calibration =
      camera_params_.constructGtsamCalibration<Calibration>();

  auto I = gtsam::traits<gtsam::Pose3>::Identity();
  gtsam::PinholeCamera<Calibration> camera(I, calibration);

  const double reprojection_error = params_.ransac_threshold_pnp;

  TrackletIds tracklets;
  TrackletIds inlier_tracklets;

  utils::Accumulatord total_repr_error, inlier_repr_error;

  const size_t& n_matches = dynamic_correspondences.size();
  for (size_t i = 0u; i < n_matches; i++) {
    const AbsolutePoseCorrespondence& corres = dynamic_correspondences.at(i);
    tracklets.push_back(corres.tracklet_id_);

    const Keypoint& kp_k = corres.cur_;
    // the landmark int the world frame at k-1
    const Landmark& w_lmk_k_1 = corres.ref_;

    // using the motion, put the lmk in the world frame at k
    const Landmark w_lmk_k = motion_model * w_lmk_k_1;
    // using camera pose, put the lmk in the camera frame at k
    const Landmark c_lmk_c = T_world_k.inverse() * w_lmk_k;

    try {
      double repr_error = camera.reprojectionError(c_lmk_c, kp_k).squaredNorm();

      total_repr_error.Add(repr_error);
      if (repr_error < reprojection_error) {
        inlier_tracklets.push_back(corres.tracklet_id_);
        inlier_repr_error.Add(repr_error);
      }
    } catch (const gtsam::CheiralityException&) {
    }
  }

  TrackletIds outliers;
  determineOutlierIds(inlier_tracklets, tracklets, outliers);

  VLOG(20) << "(Object) motion model inliers/total(error) "
           << inlier_tracklets.size() << "(" << inlier_repr_error.Mean()
           << ") / " << tracklets.size() << "(" << total_repr_error.Mean()
           << ")";

  result.best_result = motion_model;
  result.inliers = inlier_tracklets;
  result.outliers = outliers;
  result.status = TrackingStatus::VALID;
  return result;
}

// void ObjectMotionSovler::refineLocalObjectMotionEstimate(
//     Pose3SolverResult& solver_result,
//     Frame::Ptr frame_k_1,
//     Frame::Ptr frame_k,
//     ObjectId object_id,
//     const RefinementSolver& solver) const
// {
//     CHECK(solver_result.status == TrackingStatus::VALID);

//     gtsam::NonlinearFactorGraph graph;
//     gtsam::Values values;

//     // noise models are chosen arbitrarily :)
//     gtsam::SharedNoiseModel landmark_motion_noise =
//         gtsam::noiseModel::Isotropic::Sigma(3u, 0.001);

//     gtsam::SharedNoiseModel projection_noise =
//         gtsam::noiseModel::Isotropic::Sigma(2u, 2);

//     static constexpr auto k_huber_value = 0.0001;

//     //make robust (I mean, why not?)
//     landmark_motion_noise = gtsam::noiseModel::Robust::Create(
//             gtsam::noiseModel::mEstimator::Huber::Create(k_huber_value),
//             landmark_motion_noise);
//     projection_noise = gtsam::noiseModel::Robust::Create(
//             gtsam::noiseModel::mEstimator::Huber::Create(k_huber_value),
//             projection_noise);

//     const gtsam::Key pose_k_1_key =
//     CameraPoseSymbol(frame_k_1->getFrameId()); const gtsam::Key pose_k_key =
//     CameraPoseSymbol(frame_k->getFrameId()); const gtsam::Key
//     object_motion_key = ObjectMotionSymbol(object_id, frame_k->getFrameId());

//     values.insert(pose_k_1_key, frame_k_1->getPose());
//     values.insert(pose_k_key, frame_k->getPose());
//     values.insert(object_motion_key, solver_result.best_result);

//     auto gtsam_calibration = boost::make_shared<Camera::CalibrationType>(
//         frame_k_1->getFrameCamera().calibration());

//     auto pose_prior = gtsam::noiseModel::Isotropic::Sigma(6u, 0.00001);
//     graph.addPrior(pose_k_1_key, frame_k_1->getPose(), pose_prior);
//     graph.addPrior(pose_k_key, frame_k->getPose(), pose_prior);

//     utils::TimingStatsCollector timer("motion_solver.object_nlo_refinement");
//     //TODO: some might be marked outliers after update depth
//     for(TrackletId tracklet_id : solver_result.inliers) {

//         Feature::Ptr feature_k_1 = frame_k_1->at(tracklet_id);
//         Feature::Ptr feature_k = frame_k->at(tracklet_id);

//         if(!feature_k_1->usable() || !feature_k->usable()) { continue; }

//         CHECK_NOTNULL(feature_k_1);
//         CHECK_NOTNULL(feature_k);

//         CHECK(feature_k_1->hasDepth());
//         CHECK(feature_k->hasDepth());

//         const Keypoint kp_k_1 = feature_k_1->keypoint_;
//         const Keypoint kp_k = feature_k->keypoint_;

//         const gtsam::Point3 lmk_k_1_world =
//         frame_k_1->backProjectToWorld(tracklet_id); const gtsam::Point3
//         lmk_k_world = frame_k->backProjectToWorld(tracklet_id);

//         const gtsam::Point3 lmk_k_1_local =
//         frame_k_1->backProjectToCamera(tracklet_id); const gtsam::Point3
//         lmk_k_local = frame_k->backProjectToCamera(tracklet_id);

//         const gtsam::Key lmk_k_1_key =
//         DynamicLandmarkSymbol(frame_k_1->getFrameId(), tracklet_id); const
//         gtsam::Key lmk_k_key = DynamicLandmarkSymbol(frame_k->getFrameId(),
//         tracklet_id);

//         //add initial for points
//         values.insert(lmk_k_1_key, lmk_k_1_world);
//         values.insert(lmk_k_key, lmk_k_world);

//         if(solver == RefinementSolver::PointError) {
//             graph.emplace_shared<PoseToPointFactor>(
//                 pose_k_1_key, //pose key at previous frames
//                 lmk_k_1_key,
//                 lmk_k_1_local,
//                 projection_noise
//             );

//             graph.emplace_shared<PoseToPointFactor>(
//                     pose_k_key, //pose key at current frames
//                     lmk_k_key,
//                     lmk_k_local,
//                     projection_noise
//             );
//         }
//         else if(solver == RefinementSolver::ProjectionError) {
//             graph.emplace_shared<GenericProjectionFactor>(
//                     kp_k_1,
//                     projection_noise,
//                     pose_k_1_key,
//                     lmk_k_1_key,
//                     gtsam_calibration,
//                     false, false
//             );

//             graph.emplace_shared<GenericProjectionFactor>(
//                     kp_k,
//                     projection_noise,
//                     pose_k_key,
//                     lmk_k_key,
//                     gtsam_calibration,
//                     false, false
//             );
//         }

//         graph.emplace_shared<LandmarkMotionTernaryFactor>(
//             lmk_k_1_key,
//             lmk_k_key,
//             object_motion_key,
//             landmark_motion_noise
//         );

//     }

//     double error_before = graph.error(values);
//     // std::vector<double> post_errors;
//     // std::set<TrackletId> outlier_tracks;

//     gtsam::NonlinearFactorGraph mutable_graph = graph;
//     gtsam::Values optimised_values = values;

//     utils::StatsCollector("motion_solver.object_nlo_refinement_num_vars_all").AddSample(optimised_values.size());

//     gtsam::LevenbergMarquardtParams opt_params;
//     if(VLOG_IS_ON(200))
//         opt_params.verbosity =
//         gtsam::NonlinearOptimizerParams::Verbosity::ERROR;

//     optimised_values = gtsam::LevenbergMarquardtOptimizer(mutable_graph,
//     optimised_values, opt_params).optimize(); double error_after =
//     mutable_graph.error(optimised_values);
//     // post_errors.push_back(error_after);

//     // gtsam::FactorIndices outlier_factors =
//     factor_graph_tools::determineFactorOutliers<LandmarkMotionTernaryFactor>(
//     //     mutable_graph,
//     //     optimised_values
//     // );

//     // //if we have outliers, enter iteration loop
//     // if(outlier_factors.size() > 0u) {
//     //     for(size_t itr = 0; itr < 4; itr++) {

//     //         //currently removing factors from graph makes them nullptr
//     //         gtsam::NonlinearFactorGraph mutable_graph_with_null =
//     mutable_graph;
//     //         for(auto outlier_idx : outlier_factors) {
//     //             auto factor = mutable_graph_with_null.at(outlier_idx);
//     //             DynamicPointSymbol point_symbol = factor->keys()[0];
//     //             outlier_tracks.insert(point_symbol.trackletId());
//     //             mutable_graph_with_null.remove(outlier_idx);
//     //         }
//     //         //now iterate over graph and add factors that are not null to
//     ensure all factors are ok
//     //         mutable_graph.resize(0);
//     //         for (size_t i = 0; i < mutable_graph_with_null.size(); i++) {

//     //             auto factor = mutable_graph_with_null.at(i);
//     //             if(factor) {
//     //                 mutable_graph.add(factor);
//     //             }
//     //         }
//     //         // LOG(INFO) << "Removed " << outlier_factors.size() << "
//     factors on iteration: " << itr;

//     //         values.insert(object_motion_key, solver_result.best_result);
//     //         //do we use values or optimised values here?
//     //         optimised_values =
//     gtsam::LevenbergMarquardtOptimizer(mutable_graph, optimised_values,
//     opt_params).optimize();
//     //         error_after = mutable_graph.error(optimised_values);
//     //         post_errors.push_back(error_after);

//     //         outlier_factors =
//     factor_graph_tools::determineFactorOutliers<LandmarkMotionTernaryFactor>(
//     //             mutable_graph,
//     //             optimised_values
//     //         );

//     //         if(outlier_factors.size() == 0) {
//     //             break;
//     //         }
//     //     }
//     // }

//     // size_t initial_size = graph.size();
//     // size_t inlier_size = mutable_graph.size();
//     // error_after = mutable_graph.error(optimised_values);
//     LOG(INFO) << "Object Motion refinement - error before: "
//         << error_before << " error after: " << error_after;
//     //     << " with initial size " << initial_size << " inlier size " <<
//     inlier_size;

//     //recover values!
//     solver_result.best_result =
//     optimised_values.at<gtsam::Pose3>(object_motion_key);

//     //for each outlier edge, update the set of inliers
//     // for(const auto tracklet_id : outlier_tracks) {
//     //     frame_k->at(tracklet_id)->inlier_ = false;

//     // }

// }

}  // namespace dyno
