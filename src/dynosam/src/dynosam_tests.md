This file is a merged representation of the entire codebase, combined into a single document by Repomix.

# File Summary

## Purpose
This file contains a packed representation of the entire repository's contents.
It is designed to be easily consumable by AI systems for analysis, code review,
or other automated processes.

## File Format
The content is organized as follows:
1. This summary section
2. Repository information
3. Directory structure
4. Repository files (if enabled)
5. Multiple file entries, each consisting of:
  a. A header with the file path (## File: path/to/file)
  b. The full contents of the file in a code block

## Usage Guidelines
- This file should be treated as read-only. Any changes should be made to the
  original repository files, not this packed version.
- When processing this file, use the file path to distinguish
  between different files in the repository.
- Be aware that this file may contain sensitive information. Handle it with
  the same level of security as you would the original repository.

## Notes
- Some files may have been excluded based on .gitignore rules and Repomix's configuration
- Binary files are not included in this packed representation. Please refer to the Repository Structure section for a complete list of file paths, including binary files
- Files matching patterns in .gitignore are excluded
- Files matching default ignore patterns are excluded
- Files are sorted by Git change count (files with more changes are at the bottom)

# Directory Structure
```
algorithms/
  test_algorithms.cc
backend/
  test_backend_structures.cc
  test_dynamic_point_symbol.cc
  test_factors.cc
  test_landmark_quadric_factor.cc
  test_rgbd_backend.cc
  test_triangulation.cc
camera/
  ZEDCamera/
    test_ZED_camera_init.cc
  test_camera_params.cc
  test_camera.cc
core_utils/
  test_code_concepts.cc
  test_csv.cc
  test_histogram.cc
  test_numerical.cc
  test_structured_containers.cc
  test_types.cc
data/
  sensor.yaml
  tracking_params.yaml
dataproviders/
  ZEDDataProvider/
    test_ZED_data_provider_conversion_and_mask.cc
  test_dataset_provider.cc
internal/
  helpers.hpp
  simulator.cc
  simulator.hpp
  tmp_file.cc
  tmp_file.hpp
pipelines_params/
  test_params.cc
  test_pipelines.cc
thread_safety/
  test_threadsafe_imu_buffer.cc
  thread_safe_queue_tests.cc
  thread_safe_temporal_buffer_test.cc
vision_map/
  test_map.cc
  test_tools.cc
visualization/
  test_viz.cc
test_main.cc
```

# Files

## File: algorithms/test_algorithms.cc
```
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

#include "dynosam/common/Algorithms.hpp"

#include <glog/logging.h>
#include <gtest/gtest.h>
#include <exception>

#include <cmath>

using namespace dyno;

// Taken from https://www.hungarianalgorithm.com/examplehungarianalgorithm.php
TEST(Algorithms, HungarianSanityCheckSquareMatrix) {
    gtsam::Matrix44 costs;
    costs << 82, 83, 69, 92,
             77, 37, 49, 92,
             11, 69, 5, 86,
             8, 9, 98, 23;

    Eigen::VectorXi assignment;
    const double assigned_cost = internal::HungarianAlgorithm().solve(costs, assignment);

    EXPECT_EQ(assigned_cost, 140.0);

    //answer should be assignment of:
    // W1 -> J3, row0 -> col2
    // W2 -> J2, row1 -> col1
    // W3 -> J1, row2 -> col0,
    // W4 -> J4, row3 -> col3
    Eigen::VectorXd expected_assignment(4);
    expected_assignment << 2, 1, 0, 3;

    Eigen::VectorXd assignmentd = assignment.cast<double>();
    //need to cast to Eigen::VectorXd to make gtsam templating/traits happy as
    //all their matrix operations are templated on double's!
    EXPECT_TRUE(gtsam::assert_equal(expected_assignment, assignmentd));

}

TEST(Algorithms, HungarianSanityCheckMoreJobs) {
    gtsam::Matrix45 costs;
    costs << 82, 83, 69, 92, 23,
             77, 37, 49, 92, 7,
             11, 69, 5, 86, 9,
             8, 9, 98, 23, 85;

    Eigen::VectorXi assignment;
    internal::HungarianAlgorithm().solve(costs, assignment);

    Eigen::VectorXd expected_assignment(4);
    expected_assignment << 2, 1, 0, 3;

    Eigen::VectorXd assignmentd = assignment.cast<double>();
    EXPECT_EQ(assignment.rows(), 4u);

}

TEST(Algorithms, HungarianTestSimpleArgMax) {
    //a simple assignment where we want to find the MAX score
    //testing that we just use -log in the score
    gtsam::Matrix33 costs;
    costs << 3, 12, 20,
             2, 51, 4,
             33, 14, 5;

    //assignment should be
    //W1 -> J3
    //W2 -> J2
    //W3 -> J1
    //apply scalign to the costs to turn the cost function from an argmax to an argmin problem
    //which the hungrian problem sovles
    gtsam::Matrix33 loged_costs = costs.unaryExpr([](double x) { return 1.0/x * 10; });

    LOG(INFO) << loged_costs;

    Eigen::VectorXi assignment;
    internal::HungarianAlgorithm().solve(loged_costs, assignment);

    Eigen::VectorXd expected_assignment(3);
    expected_assignment << 2, 1, 0;

    Eigen::VectorXd assignmentd = assignment.cast<double>();
    EXPECT_TRUE(gtsam::assert_equal(expected_assignment, assignmentd));

}

TEST(Algorithms, OptimalAssignment) {

}
```

## File: backend/test_backend_structures.cc
```
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

#include "dynosam/backend/BackendDefinitions.hpp"
#include "dynosam/backend/FactorGraphTools.hpp"

using namespace dyno;


#include <glog/logging.h>
#include <gtest/gtest.h>


TEST(DynamicObjectSymbol, testReconstructMotionInfo) {

    gtsam::Key motion_key = ObjectMotionSymbol(12, 10);

    ObjectId recovered_object_id;
    FrameId recovered_frame_id;
    EXPECT_TRUE(reconstructMotionInfo(motion_key, recovered_object_id, recovered_frame_id));

    EXPECT_EQ(recovered_object_id, 12);
    EXPECT_EQ(recovered_frame_id, 10);

}

TEST(DynamicObjectSymbol, testReconstrucPoseInfo) {

    gtsam::Key pose_key = ObjectPoseSymbol(12, 12);

    ObjectId recovered_object_id;
    FrameId recovered_frame_id;
    EXPECT_TRUE(reconstructPoseInfo(pose_key, recovered_object_id, recovered_frame_id));

    EXPECT_EQ(recovered_object_id, 12);
    EXPECT_EQ(recovered_frame_id, 12);

}

TEST(DynamicObjectSymbol, testReconstructMotionInfoFromOtherSymbol) {

    {
        gtsam::Key motion_key = CameraPoseSymbol(10);

        ObjectId recovered_object_id;
        FrameId recovered_frame_id;
        EXPECT_FALSE(reconstructMotionInfo(motion_key, recovered_object_id, recovered_frame_id));
    }

    {
        gtsam::Key pose_key = ObjectPoseSymbol(10, 12);
        ObjectId recovered_object_id;
        FrameId recovered_frame_id;
        EXPECT_FALSE(reconstructMotionInfo(pose_key, recovered_object_id, recovered_frame_id));
    }
}

TEST(BackendDefinitions, testDynoChrExtractor) {
    gtsam::Key motion_key = ObjectMotionSymbol(12, 10);
    EXPECT_EQ(DynoChrExtractor(motion_key), kObjectMotionSymbolChar);

    gtsam::Key cam_pose_key = CameraPoseSymbol(2);
    EXPECT_EQ(DynoChrExtractor(cam_pose_key), kPoseSymbolChar);

    gtsam::Key dynamic_point_key = DynamicLandmarkSymbol(2, 10);
    EXPECT_EQ(DynoChrExtractor(dynamic_point_key), kDynamicLandmarkSymbolChar);

    gtsam::Key static_point_key = StaticLandmarkSymbol(2);
    EXPECT_EQ(DynoChrExtractor(static_point_key), kStaticLandmarkSymbolChar);
}

// TEST(FactorGraphTools, testReconstructMotionInfoFromOtherSymbol) {

//     gtsam::Key motion_key = CameraPoseSymbol(10);

//     ObjectId recovered_object_id;
//     FrameId recovered_frame_id;
//     EXPECT_FALSE(reconstructMotionInfo(motion_key, recovered_object_id, recovered_frame_id));
// }
```

## File: backend/test_dynamic_point_symbol.cc
```
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

#include "dynosam/backend/DynamicPointSymbol.hpp"


#include <glog/logging.h>
#include <gtest/gtest.h>
#include <exception>

using namespace dyno;

TEST(CantorPairingFunction, basicPair)
{
    //checking that examples provided on wikipedia are correct (ie imeplementation is good)
    //https://en.wikipedia.org/wiki/Pairing_function#Examples
    // EXPECT_EQ(CantorPairingFunction::pair({47, 32}), 3192);

}

TEST(CantorPairingFunction, basicZip)
{
    //checking that examples provided on wikipedia are correct (ie imeplementation is good)
    //https://en.wikipedia.org/wiki/Pairing_function#Examples

    // const auto z = CantorPairingFunction::unzip(1432);
    // EXPECT_EQ(z.first, 52);
    // EXPECT_EQ(z.second, 1);

}

TEST(CantorPairingFunction, testAccess)
{
    const auto x = 15;
    const auto y = 79;

    const auto z = CantorPairingFunction::pair({x, y});
    const auto result = CantorPairingFunction::depair(z);
    EXPECT_EQ(result.first, x);
    EXPECT_EQ(result.second, y);

}

TEST(CantorPairingFunction, testReconstructionSpecialCase)
{
    const auto x = 46528; //tracklet fails at runtime. Special test for this case

    const auto z = CantorPairingFunction::pair({x, 1});
    const auto result = CantorPairingFunction::depair(z);
    EXPECT_EQ(result.first, x);
    EXPECT_EQ(result.second, 1);

}


TEST(DynamicPointSymbol, testReconstruction)
{
    const auto x = 15;
    const auto y = 79;

    DynamicPointSymbol dps('m', x, y);
    gtsam::Symbol sym(dps);

    gtsam::Key key(sym);
    DynamicPointSymbol reconstructed_dps(key);
    EXPECT_EQ(dps, reconstructed_dps);
    EXPECT_EQ(x, reconstructed_dps.trackletId());
    EXPECT_EQ(y, reconstructed_dps.frameId());

}

TEST(DynamicPointSymbol, testReconstructionSpecialCase)
{
    const TrackletId bad_id = 46528; //tracklet fails at runtime. Special test for this case
    // const TrackletId bad_id = 46000; //tracklet fails at runtime. Special test for this case

    DynamicPointSymbol dps('m', bad_id, 0);
    gtsam::Symbol sym(dps);

    gtsam::Key key(sym);
    DynamicPointSymbol reconstructed_dps(key);
    EXPECT_EQ(dps, reconstructed_dps);
    EXPECT_EQ(bad_id, reconstructed_dps.trackletId());

}
```

## File: backend/test_factors.cc
```
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

#include "dynosam/factors/LandmarkMotionPoseFactor.hpp"
#include "dynosam/factors/Pose3FlowProjectionFactor.h"
#include "dynosam/factors/LandmarkMotionTernaryFactor.hpp"
#include "dynosam/utils/GtsamUtils.hpp"

#include "internal/helpers.hpp"


#include "dynosam/backend/FactorGraphTools.hpp"
#include "dynosam/backend/BackendDefinitions.hpp"

#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/base/numericalDerivative.h>

#include <glog/logging.h>
#include <gtest/gtest.h>
#include <exception>

using namespace dyno;


TEST(LandmarkMotionPoseFactor, visualiseJacobiansWithNonZeros) {

    gtsam::Pose3 L1(gtsam::Rot3::Rodrigues(-0.1, 0.2, 0.25),
                            gtsam::Point3(0.05, -0.10, 0.20));

    gtsam::Pose3 L2(gtsam::Rot3::Rodrigues(0.3, 0.2, -0.5),
                            gtsam::Point3(0.5, -0.15, 0.1));

    gtsam::Point3 p1(0.1, 2, 4);
    gtsam::Point3 p2(0.2, 3, 2);

    auto object_pose_k_1_key = ObjectPoseSymbol(0, 0);
    auto object_pose_k_key = ObjectPoseSymbol(0, 1);

    auto object_point_key_k_1 = DynamicLandmarkSymbol(0, 1);
    auto object_point_key_k = DynamicLandmarkSymbol(1, 1);

    LOG(INFO) << (std::string)object_point_key_k_1;

    gtsam::Values values;
    values.insert(object_pose_k_1_key, L1);
    values.insert(object_pose_k_key, L2);
    values.insert(object_point_key_k_1, p1);
    values.insert(object_point_key_k, p2);

    auto landmark_motion_noise = gtsam::noiseModel::Isotropic::Sigma(3u, 0.1);

    gtsam::NonlinearFactorGraph graph;
    graph.emplace_shared<LandmarkMotionPoseFactor>(
                        object_point_key_k_1,
                        object_point_key_k,
                        object_pose_k_1_key,
                        object_pose_k_key,
                        landmark_motion_noise
                    );

    NonlinearFactorGraphManager nlfgm(graph, values);

    cv::Mat block_jacobians = nlfgm.drawBlockJacobian(
        gtsam::Ordering::OrderingType::COLAMD,
        factor_graph_tools::DrawBlockJacobiansOptions::makeDynoSamOptions());

    // cv::imshow("LandmarkMotionPoseFactor block jacobians", block_jacobians);
    // cv::waitKey(0);

}

TEST(Pose3FlowProjectionFactor, testJacobians) {

    gtsam::Pose3 previous_pose = utils::createRandomAroundIdentity<gtsam::Pose3>(0.4);
    static gtsam::Pose3 kDeltaPose(gtsam::Rot3::Rodrigues(-0.1, 0.2, 0.25),
                            gtsam::Point3(0.05, -0.10, 0.20));
    gtsam::Pose3 current_pose = previous_pose * kDeltaPose;

    gtsam::Point2 kp(1.2, 2.4);
    double depth = 0.5;
    gtsam::Point2 flow(0.1, -0.3);

    auto noise = gtsam::noiseModel::Isotropic::Sigma(2u, 0.1);

    auto camera_params = dyno_testing::makeDefaultCameraParams();
    gtsam::Cal3_S2 calibration = camera_params.constructGtsamCalibration<gtsam::Cal3_S2>();

    Pose3FlowProjectionFactor<gtsam::Cal3_S2> factor(
        0,
        1,
        kp,
        depth,
        previous_pose,
        calibration,
        noise
    );

    gtsam::Matrix H1, H2;
    gtsam::Vector error = factor.evaluateError(flow, current_pose, H1, H2);

    //now do numerical jacobians
    gtsam::Matrix numerical_H1 =
            gtsam::numericalDerivative21<gtsam::Vector2, gtsam::Point2, gtsam::Pose3>(
                std::bind(&Pose3FlowProjectionFactor<gtsam::Cal3_S2>::evaluateError, &factor,
                std::placeholders::_1, std::placeholders::_2, boost::none, boost::none),
            flow, current_pose);

    gtsam::Matrix numerical_H2 =
            gtsam::numericalDerivative22<gtsam::Vector2, gtsam::Point2, gtsam::Pose3>(
                std::bind(&Pose3FlowProjectionFactor<gtsam::Cal3_S2>::evaluateError, &factor,
                std::placeholders::_1, std::placeholders::_2,boost::none, boost::none),
            flow, current_pose);

    EXPECT_TRUE(gtsam::assert_equal(H1, numerical_H1, 1e-4));
    EXPECT_TRUE(gtsam::assert_equal(H2, numerical_H2, 1e-4));



}


TEST(LandmarkMotionTernaryFactor, testJacobians) {

    gtsam::Pose3 H(gtsam::Rot3::Rodrigues(-0.1, 0.2, 0.25),
                            gtsam::Point3(0.05, -0.10, 0.20));
    gtsam::Pose3 HPerturbed = utils::perturbWithNoise<gtsam::Pose3>(H, 0.3);


   gtsam::Point3 P1(0.4, 1.0, 0.8);
   gtsam::Point3 P2 = H * P1;

    auto noise = gtsam::noiseModel::Isotropic::Sigma(3u, 0.1);

    LandmarkMotionTernaryFactor factor(0, 1, 2,noise);

    gtsam::Matrix H1, H2, H3;
    gtsam::Vector error = factor.evaluateError(P1, P2, HPerturbed, H1, H2, H3);

    //now do numerical jacobians
    gtsam::Matrix numerical_H1 =
            gtsam::numericalDerivative31<gtsam::Vector3, gtsam::Point3, gtsam::Point3, gtsam::Pose3>(
                std::bind(&LandmarkMotionTernaryFactor::evaluateError, &factor,
                std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, boost::none, boost::none, boost::none),
            P1, P2, HPerturbed);

   gtsam::Matrix numerical_H2 =
            gtsam::numericalDerivative32<gtsam::Vector3, gtsam::Point3, gtsam::Point3, gtsam::Pose3>(
                std::bind(&LandmarkMotionTernaryFactor::evaluateError, &factor,
                std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, boost::none, boost::none, boost::none),
            P1, P2, HPerturbed);

    gtsam::Matrix numerical_H3 =
            gtsam::numericalDerivative33<gtsam::Vector3, gtsam::Point3, gtsam::Point3, gtsam::Pose3>(
                std::bind(&LandmarkMotionTernaryFactor::evaluateError, &factor,
                std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, boost::none, boost::none, boost::none),
            P1, P2, HPerturbed);

    EXPECT_TRUE(gtsam::assert_equal(H1, numerical_H1));
    EXPECT_TRUE(gtsam::assert_equal(H2, numerical_H2));
    EXPECT_TRUE(gtsam::assert_equal(H3, numerical_H3));

}

TEST(LandmarkMotionTernaryFactor, testZeroError) {

    gtsam::Pose3 H(gtsam::Rot3::Rodrigues(-0.1, 0.2, 0.25),
                            gtsam::Point3(0.05, -0.10, 0.20));


   gtsam::Point3 P1(0.4, 1.0, 0.8);
   gtsam::Point3 P2 = H * P1;

    auto noise = gtsam::noiseModel::Isotropic::Sigma(3u, 0.1);

    LandmarkMotionTernaryFactor factor(0, 1, 2,noise);

    gtsam::Matrix H1, H2, H3;
    gtsam::Vector error = factor.evaluateError(P1, P2, H, H1, H2, H3);
    EXPECT_TRUE(gtsam::assert_equal(gtsam::Point3(0, 0, 0), error, 1e-4));


}
```

## File: backend/test_landmark_quadric_factor.cc
```
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

#include "dynosam/factors/LandmarkQuadricFactor.hpp"

#include <glog/logging.h>
#include <gtest/gtest.h>

using namespace dyno;

TEST(LandmarkQuadricFactor, residualBasicWithPointOnEllipse) {

    //test when radii = (3, 3, 3) (so denom = 9)
    //and then we construct a point m = 1, 2, 2 = 1 + 4 + 4 = 9
    gtsam::Point3 m_local(1, 2, 2);
    gtsam::Vector3 P(3, 3, 3);
    gtsam::Pose3 I = gtsam::Pose3::Identity();

    const gtsam::Vector1 error = LandmarkQuadricFactor::residual(
        m_local, I, P
    );
    EXPECT_TRUE(gtsam::assert_equal(error, gtsam::Vector1{0.0}));


}

TEST(LandmarkQuadricFactor, residualOnSphere) {

    //plug into ellipoide equation
    //matches distance to center of sphere + 1
    gtsam::Point3 m_local(6, 6, 3);
    gtsam::Vector3 P(3, 3, 3);
    gtsam::Pose3 I = gtsam::Pose3::Identity();

    const gtsam::Vector1 error = LandmarkQuadricFactor::residual(
        m_local, I, P
    );
    EXPECT_TRUE(gtsam::assert_equal(error, gtsam::Vector1{8.0}));


}
```

## File: backend/test_rgbd_backend.cc
```
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

#include "internal/simulator.hpp"
#include "internal/helpers.hpp"

#include "dynosam/backend/BackendPipeline.hpp"
#include "dynosam/backend/FactorGraphTools.hpp"
#include "dynosam/backend/RGBDBackendModule.hpp"

#include <gtsam/linear/GaussianEliminationTree.h>
#include <gtsam/nonlinear/ISAM2-impl.h>

#include "dynosam/factors/LandmarkMotionTernaryFactor.hpp"
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

#include <gtsam_unstable/slam/PoseToPointFactor.h>


#include <glog/logging.h>
#include <gtest/gtest.h>
#include <exception>
#include <vector>
#include <iterator>


TEST(RGBDBackendModule, constructSimpleGraph) {

    //make camera with a constant motion model starting at zero
    dyno_testing::ScenarioBody::Ptr camera = std::make_shared<dyno_testing::ScenarioBody>(
        std::make_unique<dyno_testing::ConstantMotionBodyVisitor>(
            gtsam::Pose3::Identity(),
            //motion only in x
            gtsam::Pose3(gtsam::Rot3::Identity(), gtsam::Point3(0.1, 0, 0))
        )
    );
    //needs to be at least 3 overlap so we can meet requirements in graph
    //TODO: how can we do 1 point but with lots of overlap (even infinity overlap?)
    dyno_testing::RGBDScenario scenario(
        camera,
        std::make_shared<dyno_testing::SimpleStaticPointsGenerator>(6, 3)
    );

    //add one obect
    const size_t num_points = 3;
    dyno_testing::ObjectBody::Ptr object1 = std::make_shared<dyno_testing::ObjectBody>(
        std::make_unique<dyno_testing::ConstantMotionBodyVisitor>(
            gtsam::Pose3(gtsam::Rot3::Identity(), gtsam::Point3(10, 0, 0)),
            //motion only in x
            gtsam::Pose3(gtsam::Rot3::Identity(), gtsam::Point3(0.2, 0, 0))
        ),
        std::make_unique<dyno_testing::ConstantObjectPointsVisitor>(num_points)
    );

    dyno_testing::ObjectBody::Ptr object2 = std::make_shared<dyno_testing::ObjectBody>(
        std::make_unique<dyno_testing::ConstantMotionBodyVisitor>(
            gtsam::Pose3(gtsam::Rot3::Identity(), gtsam::Point3(10, 0, 0)),
            //motion only in x
            gtsam::Pose3(gtsam::Rot3::Identity(), gtsam::Point3(0.2, 0, 0))
        ),
        std::make_unique<dyno_testing::ConstantObjectPointsVisitor>(num_points)
    );

    scenario.addObjectBody(1, object1);
    // scenario.addObjectBody(2, object2);

    dyno::Map3d2d::Ptr map = dyno::Map3d2d::create();
    // std::shared_ptr<dyno::IncrementalOptimizer<dyno::Landmark>> optimizer = std::make_shared<dyno::IncrementalOptimizer<dyno::Landmark>>();
    // std::shared_ptr<dyno_testing::DummyOptimizer<dyno::Landmark>> optimizer = std::make_shared<dyno_testing::DummyOptimizer<dyno::Landmark>>();
    // std::shared_ptr<dyno::ISAMOptimizer<dyno::Landmark>> optimizer = std::make_shared<dyno::ISAMOptimizer<dyno::Landmark>>();
    dyno::RGBDBackendModule backend(
        dyno::BackendParams().useLogger(false),
        map,
        dyno_testing::makeDefaultCameraPtr(),
        dyno::RGBDBackendModule::UpdaterType::MotionInWorld
        );

    for(size_t i = 0; i < 10; i++) {
        auto output = scenario.getOutput(i);

        std::stringstream ss;
        ss << output->T_world_camera_ << "\n";
        ss << dyno::container_to_string(output->static_landmarks_);

        // LOG(INFO) << ss.str();
        backend.spinOnce(output);

        LOG(INFO) << "Spun backend";

        // //Is this the full graph?
        // gtsam::NonlinearFactorGraph full_graph = backend.getMap()->getGraph();
        // gtsam::Values values = backend.smoother_->getLinearizationPoint();

        //get variables in this frame
        // const auto& map = backend.getMap();
        // const auto frame_k_node = map->getFrame(i);
        // const auto dyn_lmk_nodes = frame_k_node->dynamic_landmarks;
        // const auto object_nodes = frame_k_node->objects_seen;

        // gtsam::KeyVector dyn_lmks_this_frame;

        // for(const auto& dyn_lmk_node : dyn_lmk_nodes) {
        //     gtsam::Key key = dyn_lmk_node->makeDynamicKey(i);
        //     if(map->exists(key)) {
        //         dyn_lmks_this_frame.push_back(key);
        //     }
        // }

        // dyn_lmks_this_frame.push_back(map->getFrame(i)->makePoseKey());

        // gtsam::KeyVector object_motions_this_frame;

        // for(const auto& node : object_nodes) {
        //     const gtsam::Key key = frame_k_node->makeObjectMotionKey(node->getId());
        //     if(map->exists(key)) {
        //         dyn_lmks_this_frame.push_back(key);
        //     }
        // }


        // gtsam::VariableIndex affectedFactorsVarIndex(full_graph);
        // gtsam::Ordering order = gtsam::Ordering::ColamdConstrainedLast(affectedFactorsVarIndex, dyn_lmks_this_frame);
        // auto linearized = full_graph.linearize(values);

        // auto bayesTree =
        //     gtsam::ISAM2JunctionTree(
        //         gtsam::GaussianEliminationTree(*linearized, affectedFactorsVarIndex, order))
        //         .eliminate(gtsam::EliminatePreferCholesky)
        //         .first;

        // LOG(INFO) << bayesTree->roots().size();
        // LOG(WARNING) << "Number nnz bayes tree " << bayesTree->roots().at(0)->calculate_nnz();
        // LOG(WARNING) << "Number nnz isam2 tree " << optimizer->getSmoother().roots().at(0)->calculate_nnz();

        // // bayesTree->saveGraph(dyno::getOutputFilePath("elimated_tree.dot"), dyno::DynoLikeKeyFormatter);
        // dyno::factor_graph_tools::saveBayesTree(*bayesTree, dyno::getOutputFilePath("elimated_tree.dot"), dyno::DynoLikeKeyFormatter);

        // const auto& dynosam2_result = optimizer->getResult();

        // gtsam::FastMap<gtsam::Key, std::string> coloured_affected_keys;
        // for(const auto& key : dynosam2_result.reeliminatedKeys) {
        //     coloured_affected_keys.insert2(key, "red");
        // }

        // // const auto& smoother = optimizer->getSmoother().bayesTree();
        // const auto& smoother = optimizer->getSmoother();

        // // smoother.calculateBestEstimate();
        // if(smoother.roots().empty()) {
        //     continue;
        // }

        // dyno::factor_graph_tools::saveBayesTree(
        //     smoother,
        //     dyno::getOutputFilePath("rgbd_bayes_tree_" + std::to_string(i) + ".dot"),
        //     dyno::DynoLikeKeyFormatter,
        //     coloured_affected_keys);

        // {
        //     gtsam::KeySet marginalize_keys = dyno::factor_graph_tools::travsersal::getLeafKeys(smoother);
        //     gtsam::PrintKeySet(marginalize_keys, "Leaf keys", dyno::DynoLikeKeyFormatter);

        //     gtsam::FastMap<gtsam::Key, std::string> coloured_affected_keys;
        //     for(const auto& key : marginalize_keys) {
        //         coloured_affected_keys.insert2(key, "blue");
        //     }

        //     dyno::factor_graph_tools::saveBayesTree(
        //         smoother,
        //         dyno::getOutputFilePath("rgbd_bayes_tree_leaf.dot"),
        //         dyno::DynoLikeKeyFormatter,
        //         coloured_affected_keys);

        // }

        // smoother.getFactorsUnsafe().saveGraph(dyno::getOutputFilePath("smoother_graph_" + std::to_string(i) + ".dot"), dyno::DynoLikeKeyFormatter);

        // backend.saveGraph("rgbd_graph_" + std::to_string(i) + ".dot");
        // backend.saveTree("rgbd_bayes_tree_" + std::to_string(i) + ".dot");
    }

    gtsam::NonlinearFactorGraph full_graph = backend.new_updater_->getGraph();
    full_graph.saveGraph(dyno::getOutputFilePath("construct_simple_graph_test.dot"), dyno::DynoLikeKeyFormatter);

    {
        const auto[delayed_values, delayed_graph] = backend.constructGraph(2, 6, true);
        delayed_graph.saveGraph(dyno::getOutputFilePath("construct_simple_delayed_graph_test_2_6.dot"), dyno::DynoLikeKeyFormatter);
    }

    {
    const auto[delayed_values, delayed_graph] = backend.constructGraph(5, 9, true);
    delayed_graph.saveGraph(dyno::getOutputFilePath("construct_simple_delayed_graph_test_5_9.dot"), dyno::DynoLikeKeyFormatter);
    }

    // gtsam::LevenbergMarquardtParams opt_params;
    // opt_params.verbosity = gtsam::NonlinearOptimizerParams::Verbosity::ERROR;
    // // opt_params.
    // try {
    //     gtsam::Values opt_values = gtsam::LevenbergMarquardtOptimizer(delayed_graph, delayed_values, opt_params).optimize();
    // }
    // catch(const gtsam::ValuesKeyDoesNotExist& e) {
    //     LOG(INFO) << "Key does not exist in the values " <<  dyno::DynoLikeKeyFormatter(e.key());
    // }


    //graph depends on optimzier used
    //dummy one will += new factors so should be the full graph
    // gtsam::NonlinearFactorGraph full_graph = optimizer->getFactors();
    // full_graph.saveGraph(dyno::getOutputFilePath("construct_simple_graph_test.dot"), dyno::DynoLikeKeyFormatter);

    // backend.saveTree();





    // auto graph = backend.getMap()->getGraph();
    // auto estimates = backend.getMap()->getValues();

    // gtsam::GaussianFactorGraph factors = *graph.linearize(estimates);
    // gtsam::VariableIndex affectedFactorsVarIndex(factors);
    // //NOTE: NOT the same as in sam2 because of the affectedFactorsVarIndex
    // //maybe we can reconstruct the affectedFactorsVarIndex by looking at the factor index's and making a factor graph
    // //from this? This needs to be the factors that are affected
    // const gtsam::Ordering ordering =
    //   gtsam::Ordering::ColamdConstrained(affectedFactorsVarIndex, gtsam::FastMap<gtsam::Key, int>{});

    // gtsam::GaussianEliminationTree etree(factors, affectedFactorsVarIndex, ordering);

    // auto bayesTree = gtsam::ISAM2JunctionTree(etree)
    //                    .eliminate(gtsam::EliminatePreferCholesky)
    //                    .first;
    // bayesTree->saveGraph(dyno::getOutputFilePath("elimated_tree.dot"), dyno::DynoLikeKeyFormatter);
    // // etree.sa();

    // auto gfg = graph.linearize(estimates);

    // gtsam::Ordering isam2_ordering(dyno::factor_graph_tools::travsersal::getEliminatonOrder(*backend.smoother_));

    // //this ordering is not right. how do we get the ordering used by the bayes tree?
    // //OH DUH, the bayes tree IS the order!!! So which tree traversal algorithm do we use?
    // //Do we not start at the roots and check the frontal variables?
    // dyno::factor_graph_tools::computeRFactor(gfg, isam2_ordering);



}




TEST(RGBDBackendModule, testCliques) {
    using namespace dyno;
    //the simplest dynamic graph
    gtsam::NonlinearFactorGraph graph;
    gtsam::Values initial;

    gtsam::Pose3 H_0_1 = gtsam::Pose3(gtsam::Rot3::Rodrigues(0.3,0.2,0.1), gtsam::Point3 (0, 1, 1));
    gtsam::Pose3 H_1_2 = gtsam::Pose3(gtsam::Rot3::Rodrigues(0.5,0.1,0.1), gtsam::Point3 (1, 1.5, 1));
    gtsam::Pose3 H_0_2 = H_0_1 * H_1_2;

    gtsam::Point3 dyn_point_1_world = gtsam::Point3(2, 1, 3);
    gtsam::Point3 dyn_point_2_world = gtsam::Point3(1, 1, 3);
    gtsam::Point3 dyn_point_3_world = gtsam::Point3(3, 0.5, 2);

    static gtsam::Point3 p0 = gtsam::Point3(1,3,4);
    static gtsam::Rot3   R0 = gtsam::Rot3::Expmap( ( gtsam::Vector(3) << 0.2,0.1,0.12 ).finished() );
    static gtsam::Point3 p1 = gtsam::Point3(1,2,1);
    static gtsam::Rot3   R1 = gtsam::Rot3::Expmap( ( gtsam::Vector(3) << 0.1,0.2,1.570796 ).finished() );
    static gtsam::Point3 p2 = gtsam::Point3(2,2,1);
    static gtsam::Rot3   R2 = gtsam::Rot3::Expmap( ( gtsam::Vector(3) << 0.2,0.3,3.141593 ).finished() );
    static gtsam::Point3 p3 = gtsam::Point3(-1,1,0);
    static gtsam::Rot3   R3 = gtsam::Rot3::Expmap( ( gtsam::Vector(3) << 0.1,0.4,4.712389 ).finished() );

    static gtsam::Pose3 pose0 = gtsam::Pose3(R0,p0);
    static gtsam::Pose3 pose1 = gtsam::Pose3(R1,p1);
    static gtsam::Pose3 pose2 = gtsam::Pose3(R2,p2);
    static gtsam::Pose3 pose3 = gtsam::Pose3(R3,p3);

    auto landmark_noise = gtsam::noiseModel::Isotropic::Sigma(3u, 10);

    //static point seen at frames 0, 1 and 2
    graph.emplace_shared<gtsam::PoseToPointFactor<gtsam::Pose3, Landmark>>(
            CameraPoseSymbol(0),
            StaticLandmarkSymbol(0),
            gtsam::Point3(1, 2, 3),
            landmark_noise
        );
    graph.emplace_shared<gtsam::PoseToPointFactor<gtsam::Pose3, Landmark>>(
            CameraPoseSymbol(1),
            StaticLandmarkSymbol(0),
            gtsam::Point3(2, 2, 3),
            landmark_noise
        );

    graph.emplace_shared<gtsam::PoseToPointFactor<gtsam::Pose3, Landmark>>(
            CameraPoseSymbol(2),
            StaticLandmarkSymbol(0),
            gtsam::Point3(3, 2, 3),
            landmark_noise
        );

    //motion between frames 0 and 1
    //add motion factor for the 3 tracklet with tracklet id = 1, 2, 3
    graph.emplace_shared<LandmarkMotionTernaryFactor>(
        DynamicLandmarkSymbol(0, 1),
        DynamicLandmarkSymbol(1, 1),
        ObjectMotionSymbol(1, 1),
        landmark_noise
    );

    //motion between frames 1 and 2
    graph.emplace_shared<LandmarkMotionTernaryFactor>(
        DynamicLandmarkSymbol(1, 1),
        DynamicLandmarkSymbol(2, 1),
        ObjectMotionSymbol(1, 2),
        landmark_noise
    );

    //tracklet 2
    graph.emplace_shared<LandmarkMotionTernaryFactor>(
        DynamicLandmarkSymbol(0, 2),
        DynamicLandmarkSymbol(1, 2),
        ObjectMotionSymbol(1, 1),
        landmark_noise
    );
    //motion between frames 1 and 2
    graph.emplace_shared<LandmarkMotionTernaryFactor>(
        DynamicLandmarkSymbol(1, 2),
        DynamicLandmarkSymbol(2, 2),
        ObjectMotionSymbol(1, 2),
        landmark_noise
    );


    //tracklet 3
    graph.emplace_shared<LandmarkMotionTernaryFactor>(
        DynamicLandmarkSymbol(0, 3),
        DynamicLandmarkSymbol(1, 3),
        ObjectMotionSymbol(1, 1),
        landmark_noise
    );
    //motion between frames 1 and 2
    graph.emplace_shared<LandmarkMotionTernaryFactor>(
        DynamicLandmarkSymbol(1, 3),
        DynamicLandmarkSymbol(2, 3),
        ObjectMotionSymbol(1, 2),
        landmark_noise
    );

    //tracklet 1
    //add dynamic point obs frame 0
    graph.emplace_shared<gtsam::PoseToPointFactor<gtsam::Pose3, Landmark>>(
        CameraPoseSymbol(0),
        DynamicLandmarkSymbol(0, 1),
        pose0.inverse() * dyn_point_1_world,
        landmark_noise
    );

     //add dynamic point obs frame 1
    graph.emplace_shared<gtsam::PoseToPointFactor<gtsam::Pose3, Landmark>>(
            CameraPoseSymbol(1),
            DynamicLandmarkSymbol(1, 1),
            pose1.inverse() * H_0_1 * dyn_point_1_world,
            landmark_noise
        );

     //add dynamic point obs frame 2
    graph.emplace_shared<gtsam::PoseToPointFactor<gtsam::Pose3, Landmark>>(
            CameraPoseSymbol(2),
            DynamicLandmarkSymbol(2, 1),
            pose2.inverse() * H_0_2 * dyn_point_1_world,
            landmark_noise
    );

    //tracklet 2
    //add dynamic point obs frame 0
    graph.emplace_shared<gtsam::PoseToPointFactor<gtsam::Pose3, Landmark>>(
        CameraPoseSymbol(0),
        DynamicLandmarkSymbol(0, 2),
        pose0.inverse() * dyn_point_2_world,
        landmark_noise
    );

     //add dynamic point obs frame 1
    graph.emplace_shared<gtsam::PoseToPointFactor<gtsam::Pose3, Landmark>>(
            CameraPoseSymbol(1),
            DynamicLandmarkSymbol(1, 2),
            pose1.inverse() * H_0_1 * dyn_point_2_world,
            landmark_noise
        );

     //add dynamic point obs frame 2
    graph.emplace_shared<gtsam::PoseToPointFactor<gtsam::Pose3, Landmark>>(
            CameraPoseSymbol(2),
            DynamicLandmarkSymbol(2, 2),
            pose2.inverse() * H_0_2 * dyn_point_2_world,
            landmark_noise
    );


    //tracklet 3
    //add dynamic point obs frame 0
    graph.emplace_shared<gtsam::PoseToPointFactor<gtsam::Pose3, Landmark>>(
        CameraPoseSymbol(0),
        DynamicLandmarkSymbol(0, 3),
        pose0.inverse() * dyn_point_3_world,
        landmark_noise
    );

     //add dynamic point obs frame 1
    graph.emplace_shared<gtsam::PoseToPointFactor<gtsam::Pose3, Landmark>>(
            CameraPoseSymbol(1),
            DynamicLandmarkSymbol(1, 3),
            pose1.inverse() * H_0_1 * dyn_point_3_world,
            landmark_noise
        );

     //add dynamic point obs frame 2
    graph.emplace_shared<gtsam::PoseToPointFactor<gtsam::Pose3, Landmark>>(
            CameraPoseSymbol(2),
            DynamicLandmarkSymbol(2, 3),
            pose2.inverse() * H_0_2 * dyn_point_3_world,
            landmark_noise
    );


    static gtsam::SharedNoiseModel pose_model(gtsam::noiseModel::Isotropic::Sigma(6, 0.1));

    // add two poses
    graph.add(gtsam::BetweenFactor<gtsam::Pose3>(CameraPoseSymbol(0), CameraPoseSymbol(1), pose0.between(pose1), pose_model));
    graph.add(gtsam::BetweenFactor<gtsam::Pose3>(CameraPoseSymbol(1), CameraPoseSymbol(2), pose1.between(pose2), pose_model));

    // add prior on first pose
    graph.addPrior(CameraPoseSymbol(0), pose0, pose_model);

    //now add inital values

    //add static point
    initial.insert(StaticLandmarkSymbol(0), gtsam::Point3(0, 1, 1));

    //add dynamic points

    //frame 0
    initial.insert(DynamicLandmarkSymbol(0, 1), dyn_point_1_world);
    initial.insert(DynamicLandmarkSymbol(0, 2), dyn_point_2_world);
    initial.insert(DynamicLandmarkSymbol(0, 3), dyn_point_3_world);

    //frame 1
    initial.insert(DynamicLandmarkSymbol(1, 1), H_0_1 * dyn_point_1_world);
    initial.insert(DynamicLandmarkSymbol(1, 2), H_0_1 * dyn_point_2_world);
    initial.insert(DynamicLandmarkSymbol(1, 3), H_0_1 * dyn_point_3_world);

    //frame 2
    initial.insert(DynamicLandmarkSymbol(2, 1), H_0_2 * dyn_point_1_world);
    initial.insert(DynamicLandmarkSymbol(2, 2), H_0_2 * dyn_point_2_world);
    initial.insert(DynamicLandmarkSymbol(2, 3), H_0_2 * dyn_point_3_world);

    //add two motions
    initial.insert(ObjectMotionSymbol(1, 1), gtsam::Pose3::Identity());
    initial.insert(ObjectMotionSymbol(1, 2), gtsam::Pose3::Identity());

    //add three poses
    initial.insert(CameraPoseSymbol(0), pose0);
    initial.insert(CameraPoseSymbol(1), pose1);
    initial.insert(CameraPoseSymbol(2), pose2);

    dyno::NonlinearFactorGraphManager nlfgm(graph, initial);
    nlfgm.writeDynosamGraphFile(dyno::getOutputFilePath("test_graph.g2o"));

    // graph.saveGraph(dyno::getOutputFilePath("small_graph.dot"), dyno::DynoLikeKeyFormatter);
    gtsam::ISAM2BayesTree::shared_ptr bayesTree = nullptr;
    {//
        // gtsam::ISAM2 isam2;
        gtsam::VariableIndex affectedFactorsVarIndex(graph);
        gtsam::Ordering order = gtsam::Ordering::Colamd(affectedFactorsVarIndex);
        auto linearized = graph.linearize(initial);

        bayesTree =
        gtsam::ISAM2JunctionTree(
            gtsam::GaussianEliminationTree(*linearized, affectedFactorsVarIndex, order))
            .eliminate(gtsam::EliminatePreferCholesky)
            .first;

        bayesTree->saveGraph(dyno::getOutputFilePath("elimated_tree.dot"), dyno::DynoLikeKeyFormatter);
    }

    // {

    //     gtsam::ISAM2UpdateParams isam_update_params;

    //     gtsam::FastMap<gtsam::Key, int> constrainedKeys;
    //     //this includes the motions, where do we want these?
    //     //maybe BETWEEN the dynnamic keys
    //     //we want motions always on the right but impossible since parent?
    //     for(const auto& [keys, value] : initial) {
    //         constrainedKeys.insert2(keys, 1);
    //     }
    //     //put previous dynamic keys lower in the graph
    //     constrainedKeys.insert2(DynamicLandmarkSymbol(0, 1), 0);
    //     constrainedKeys.insert2(DynamicLandmarkSymbol(1, 1), 0);

    //     constrainedKeys.insert2(DynamicLandmarkSymbol(0, 2), 0);
    //     constrainedKeys.insert2(DynamicLandmarkSymbol(1, 2), 0);

    //     constrainedKeys.insert2(DynamicLandmarkSymbol(0, 2), 0);
    //     constrainedKeys.insert2(DynamicLandmarkSymbol(1, 2), 0);

    //     ///put current keys later in the graph
    //     constrainedKeys.insert2(DynamicLandmarkSymbol(2, 1), 2);
    //     constrainedKeys.insert2(DynamicLandmarkSymbol(2, 2), 2);
    //     constrainedKeys.insert2(DynamicLandmarkSymbol(2, 3), 2);

    //     isam_update_params.constrainedKeys = constrainedKeys;
    //     isam2.update(graph, initial, isam_update_params);
    // }

    // isam2.saveGraph(dyno::getOutputFilePath("small_tree_original.dot"), dyno::DynoLikeKeyFormatter);

    //add new factors to  motion
    //the simplest dynamic graph
    gtsam::NonlinearFactorGraph new_graph;
    gtsam::Values new_values;

    gtsam::Pose3 H_2_3 = gtsam::Pose3(gtsam::Rot3::Rodrigues(0.4,0.15,0.1), gtsam::Point3 (0, 3, 1));
    gtsam::Pose3 H_0_3 = H_0_2 * H_2_3;

    //add dynamic obs
    new_graph.emplace_shared<LandmarkMotionTernaryFactor>(
        DynamicLandmarkSymbol(2, 1),
        DynamicLandmarkSymbol(3, 1),
        ObjectMotionSymbol(1, 3),
        landmark_noise
    );

     new_graph.emplace_shared<LandmarkMotionTernaryFactor>(
        DynamicLandmarkSymbol(2, 2),
        DynamicLandmarkSymbol(3, 2),
        ObjectMotionSymbol(1, 3),
        landmark_noise
    );

    new_graph.emplace_shared<LandmarkMotionTernaryFactor>(
        DynamicLandmarkSymbol(2, 3),
        DynamicLandmarkSymbol(3, 3),
        ObjectMotionSymbol(1, 3),
        landmark_noise
    );

    //add pose-point constraints
    new_graph.emplace_shared<gtsam::PoseToPointFactor<gtsam::Pose3, Landmark>>(
        CameraPoseSymbol(3),
        DynamicLandmarkSymbol(3, 1),
        pose3.inverse() * H_0_3 * dyn_point_1_world,
        landmark_noise
    );
    new_graph.emplace_shared<gtsam::PoseToPointFactor<gtsam::Pose3, Landmark>>(
            CameraPoseSymbol(3),
            DynamicLandmarkSymbol(3,2),
            pose3.inverse() * H_0_3 * dyn_point_2_world,
            landmark_noise
        );

    new_graph.emplace_shared<gtsam::PoseToPointFactor<gtsam::Pose3, Landmark>>(
            CameraPoseSymbol(3),
            DynamicLandmarkSymbol(3, 3),
            pose3.inverse() * H_0_3 * dyn_point_3_world,
            landmark_noise
    );

    //add static obs
    new_graph.emplace_shared<gtsam::PoseToPointFactor<gtsam::Pose3, Landmark>>(
            CameraPoseSymbol(3),
            StaticLandmarkSymbol(0),
            gtsam::Point3(4, 2, 3),
            landmark_noise
        );

    //add odom
    new_graph.add(gtsam::BetweenFactor<gtsam::Pose3>(CameraPoseSymbol(2), CameraPoseSymbol(3), pose2.between(pose3), pose_model));

    //add initials
    new_values.insert(DynamicLandmarkSymbol(3, 1), H_0_3 * dyn_point_1_world);
    new_values.insert(DynamicLandmarkSymbol(3, 2), H_0_3 * dyn_point_2_world);
    new_values.insert(DynamicLandmarkSymbol(3, 3), H_0_3 * dyn_point_3_world);
    new_values.insert(ObjectMotionSymbol(1, 3), gtsam::Pose3::Identity());

    new_values.insert(CameraPoseSymbol(3), pose3);

    {
        gtsam::NonlinearFactorGraph full_graph(graph);
        full_graph.add_factors(new_graph);

        gtsam::Values all_values(initial);
        all_values.insert(new_values);

        gtsam::VariableIndex affectedFactorsVarIndex(full_graph);
        gtsam::Ordering order = gtsam::Ordering::ColamdConstrainedLast(affectedFactorsVarIndex, full_graph.keyVector(), true);
        auto linearized = full_graph.linearize(all_values);

        bayesTree =
        gtsam::ISAM2JunctionTree(
            gtsam::GaussianEliminationTree(*linearized, affectedFactorsVarIndex, order))
            .eliminate(gtsam::EliminatePreferCholesky)
            .first;

        bayesTree->saveGraph(dyno::getOutputFilePath("elimated_tree_1.dot"), dyno::DynoLikeKeyFormatter);
    }

    // gtsam::NonlinearFactorGraph all = graph;
    // all.add_factors(new_graph);
    // all.saveGraph(dyno::getOutputFilePath("small_graph_updated.dot"), dyno::DynoLikeKeyFormatter);

    //  {

    //     gtsam::ISAM2UpdateParams isam_update_params;

    //     gtsam::FastMap<gtsam::Key, int> constrainedKeys;
    //     //this includes the motions, where do we want these?
    //     //maybe BETWEEN the dynnamic keys
    //     //we want motions always on the right but impossible since parent?
    //     for(const auto& [keys, value] : new_values) {
    //         constrainedKeys.insert2(keys, 1);
    //     }
    //     constrainedKeys.insert2(DynamicLandmarkSymbol(2, 1), 0);
    //     constrainedKeys.insert2(DynamicLandmarkSymbol(2, 2), 0);
    //     constrainedKeys.insert2(DynamicLandmarkSymbol(2, 3), 0);

    //     ///put current keys later in the graph
    //     constrainedKeys.insert2(DynamicLandmarkSymbol(3, 1), 2);
    //     constrainedKeys.insert2(DynamicLandmarkSymbol(3, 2), 2);
    //     constrainedKeys.insert2(DynamicLandmarkSymbol(3, 3), 2);

    //     isam_update_params.constrainedKeys = constrainedKeys;
    //     isam2.update(new_graph, new_values, isam_update_params);
    // }

    // isam2.saveGraph(dyno::getOutputFilePath("small_tree_updated.dot"), dyno::DynoLikeKeyFormatter);





}


TEST(RGBDBackendModule, writeOutSimpleGraph) {
    using namespace dyno;
    //the simplest dynamic graph
    gtsam::NonlinearFactorGraph graph;
    gtsam::Values initial;

    gtsam::Pose3 H_0_1 = gtsam::Pose3(gtsam::Rot3::Rodrigues(0.3,0.2,0.1), gtsam::Point3 (0, 1, 1));
    gtsam::Pose3 H_1_2 = gtsam::Pose3(gtsam::Rot3::Rodrigues(0.5,0.1,0.1), gtsam::Point3 (1, 1.5, 1));
    gtsam::Pose3 H_0_2 = H_0_1 * H_1_2;

    gtsam::Point3 dyn_point_1_world = gtsam::Point3(2, 1, 3);
    gtsam::Point3 dyn_point_2_world = gtsam::Point3(1, 1, 3);
    gtsam::Point3 dyn_point_3_world = gtsam::Point3(3, 0.5, 2);

    static gtsam::Point3 p0 = gtsam::Point3(1,3,4);
    static gtsam::Rot3   R0 = gtsam::Rot3::Expmap( ( gtsam::Vector(3) << 0.2,0.1,0.12 ).finished() );
    static gtsam::Point3 p1 = gtsam::Point3(1,2,1);
    static gtsam::Rot3   R1 = gtsam::Rot3::Expmap( ( gtsam::Vector(3) << 0.1,0.2,1.570796 ).finished() );
    static gtsam::Point3 p2 = gtsam::Point3(2,2,1);
    static gtsam::Rot3   R2 = gtsam::Rot3::Expmap( ( gtsam::Vector(3) << 0.2,0.3,3.141593 ).finished() );
    static gtsam::Point3 p3 = gtsam::Point3(-1,1,0);
    static gtsam::Rot3   R3 = gtsam::Rot3::Expmap( ( gtsam::Vector(3) << 0.1,0.4,4.712389 ).finished() );

    static gtsam::Pose3 pose0 = gtsam::Pose3(R0,p0);
    static gtsam::Pose3 pose1 = gtsam::Pose3(R1,p1);
    static gtsam::Pose3 pose2 = gtsam::Pose3(R2,p2);
    static gtsam::Pose3 pose3 = gtsam::Pose3(R3,p3);

    auto landmark_noise = gtsam::noiseModel::Isotropic::Sigma(3u, 10);

    //static point seen at frames 0, 1 and 2
    graph.emplace_shared<gtsam::PoseToPointFactor<gtsam::Pose3, Landmark>>(
            CameraPoseSymbol(0),
            StaticLandmarkSymbol(0),
            gtsam::Point3(1, 2, 3),
            landmark_noise
        );
    graph.emplace_shared<gtsam::PoseToPointFactor<gtsam::Pose3, Landmark>>(
            CameraPoseSymbol(1),
            StaticLandmarkSymbol(0),
            gtsam::Point3(2, 2, 3),
            landmark_noise
        );

    graph.emplace_shared<gtsam::PoseToPointFactor<gtsam::Pose3, Landmark>>(
            CameraPoseSymbol(2),
            StaticLandmarkSymbol(0),
            gtsam::Point3(3, 2, 3),
            landmark_noise
        );

    //motion between frames 0 and 1
    //add motion factor for the 3 tracklet with tracklet id = 1, 2, 3
    graph.emplace_shared<LandmarkMotionTernaryFactor>(
        DynamicLandmarkSymbol(0, 1),
        DynamicLandmarkSymbol(1, 1),
        ObjectMotionSymbol(1, 1),
        landmark_noise
    );

    //motion between frames 1 and 2
    graph.emplace_shared<LandmarkMotionTernaryFactor>(
        DynamicLandmarkSymbol(1, 1),
        DynamicLandmarkSymbol(2, 1),
        ObjectMotionSymbol(1, 2),
        landmark_noise
    );

    //tracklet 2
    graph.emplace_shared<LandmarkMotionTernaryFactor>(
        DynamicLandmarkSymbol(0, 2),
        DynamicLandmarkSymbol(1, 2),
        ObjectMotionSymbol(1, 1),
        landmark_noise
    );
    //motion between frames 1 and 2
    graph.emplace_shared<LandmarkMotionTernaryFactor>(
        DynamicLandmarkSymbol(1, 2),
        DynamicLandmarkSymbol(2, 2),
        ObjectMotionSymbol(1, 2),
        landmark_noise
    );


    //tracklet 3
    graph.emplace_shared<LandmarkMotionTernaryFactor>(
        DynamicLandmarkSymbol(0, 3),
        DynamicLandmarkSymbol(1, 3),
        ObjectMotionSymbol(1, 1),
        landmark_noise
    );
    //motion between frames 1 and 2
    graph.emplace_shared<LandmarkMotionTernaryFactor>(
        DynamicLandmarkSymbol(1, 3),
        DynamicLandmarkSymbol(2, 3),
        ObjectMotionSymbol(1, 2),
        landmark_noise
    );

    //tracklet 1
    //add dynamic point obs frame 0
    graph.emplace_shared<gtsam::PoseToPointFactor<gtsam::Pose3, Landmark>>(
        CameraPoseSymbol(0),
        DynamicLandmarkSymbol(0, 1),
        pose0.inverse() * dyn_point_1_world,
        landmark_noise
    );

     //add dynamic point obs frame 1
    graph.emplace_shared<gtsam::PoseToPointFactor<gtsam::Pose3, Landmark>>(
            CameraPoseSymbol(1),
            DynamicLandmarkSymbol(1, 1),
            pose1.inverse() * H_0_1 * dyn_point_1_world,
            landmark_noise
        );

     //add dynamic point obs frame 2
    graph.emplace_shared<gtsam::PoseToPointFactor<gtsam::Pose3, Landmark>>(
            CameraPoseSymbol(2),
            DynamicLandmarkSymbol(2, 1),
            pose2.inverse() * H_0_2 * dyn_point_1_world,
            landmark_noise
    );

    //tracklet 2
    //add dynamic point obs frame 0
    graph.emplace_shared<gtsam::PoseToPointFactor<gtsam::Pose3, Landmark>>(
        CameraPoseSymbol(0),
        DynamicLandmarkSymbol(0, 2),
        pose0.inverse() * dyn_point_2_world,
        landmark_noise
    );

     //add dynamic point obs frame 1
    graph.emplace_shared<gtsam::PoseToPointFactor<gtsam::Pose3, Landmark>>(
            CameraPoseSymbol(1),
            DynamicLandmarkSymbol(1, 2),
            pose1.inverse() * H_0_1 * dyn_point_2_world,
            landmark_noise
        );

     //add dynamic point obs frame 2
    graph.emplace_shared<gtsam::PoseToPointFactor<gtsam::Pose3, Landmark>>(
            CameraPoseSymbol(2),
            DynamicLandmarkSymbol(2, 2),
            pose2.inverse() * H_0_2 * dyn_point_2_world,
            landmark_noise
    );


    //tracklet 3
    //add dynamic point obs frame 0
    graph.emplace_shared<gtsam::PoseToPointFactor<gtsam::Pose3, Landmark>>(
        CameraPoseSymbol(0),
        DynamicLandmarkSymbol(0, 3),
        pose0.inverse() * dyn_point_3_world,
        landmark_noise
    );

     //add dynamic point obs frame 1
    graph.emplace_shared<gtsam::PoseToPointFactor<gtsam::Pose3, Landmark>>(
            CameraPoseSymbol(1),
            DynamicLandmarkSymbol(1, 3),
            pose1.inverse() * H_0_1 * dyn_point_3_world,
            landmark_noise
        );

     //add dynamic point obs frame 2
    graph.emplace_shared<gtsam::PoseToPointFactor<gtsam::Pose3, Landmark>>(
            CameraPoseSymbol(2),
            DynamicLandmarkSymbol(2, 3),
            pose2.inverse() * H_0_2 * dyn_point_3_world,
            landmark_noise
    );


    static gtsam::SharedNoiseModel pose_model(gtsam::noiseModel::Isotropic::Sigma(6, 0.1));

    // add two poses
    graph.add(gtsam::BetweenFactor<gtsam::Pose3>(CameraPoseSymbol(0), CameraPoseSymbol(1), pose0.between(pose1), pose_model));
    graph.add(gtsam::BetweenFactor<gtsam::Pose3>(CameraPoseSymbol(1), CameraPoseSymbol(2), pose1.between(pose2), pose_model));

    // add prior on first pose
    graph.addPrior(CameraPoseSymbol(0), pose0, pose_model);

    //now add inital values

    //add static point
    initial.insert(StaticLandmarkSymbol(0), gtsam::Point3(0, 1, 1));

    //add dynamic points

    //frame 0
    initial.insert(DynamicLandmarkSymbol(0, 1), dyn_point_1_world);
    initial.insert(DynamicLandmarkSymbol(0, 2), dyn_point_2_world);
    initial.insert(DynamicLandmarkSymbol(0, 3), dyn_point_3_world);

    //frame 1
    initial.insert(DynamicLandmarkSymbol(1, 1), H_0_1 * dyn_point_1_world);
    initial.insert(DynamicLandmarkSymbol(1, 2), H_0_1 * dyn_point_2_world);
    initial.insert(DynamicLandmarkSymbol(1, 3), H_0_1 * dyn_point_3_world);

    //frame 2
    initial.insert(DynamicLandmarkSymbol(2, 1), H_0_2 * dyn_point_1_world);
    initial.insert(DynamicLandmarkSymbol(2, 2), H_0_2 * dyn_point_2_world);
    initial.insert(DynamicLandmarkSymbol(2, 3), H_0_2 * dyn_point_3_world);

    //add two motions
    initial.insert(ObjectMotionSymbol(1, 1), gtsam::Pose3::Identity());
    initial.insert(ObjectMotionSymbol(1, 2), gtsam::Pose3::Identity());

    //add three poses
    initial.insert(CameraPoseSymbol(0), pose0);
    initial.insert(CameraPoseSymbol(1), pose1);
    initial.insert(CameraPoseSymbol(2), pose2);

    dyno::NonlinearFactorGraphManager nlfgm(graph, initial);
    // nlfgm.writeDynosamGraphFile(dyno::getOutputFilePath("test_graph.g2o"));

    graph.saveGraph(dyno::getOutputFilePath("simple_dynamic_graph.dot"), dyno::DynoLikeKeyFormatter);
    gtsam::GaussianFactorGraph::shared_ptr linearised_graph = nlfgm.linearize();
    gtsam::JacobianFactor jacobian_factor(*linearised_graph);
    gtsam::Matrix information = jacobian_factor.information();

    LOG(INFO) << information.rows() << " " << information.cols();

    writeMatrixWithPythonFormat(information, dyno::getOutputFilePath("simple_dynamic_graph_information.txt"));



}
```

## File: backend/test_triangulation.cc
```
/*
 *   Copyright (c) 2023 ACFR-RPG, University of Sydney, Yiduo Wang (yiduo.wang@sydney.edu.au)
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
#include "dynosam/backend/MonoBackendTools.hpp"
#include "dynosam/common/Types.hpp"

#include <gtsam/base/Vector.h>
#include <gtest/gtest.h>

using namespace dyno;


TEST(MonoBackendTools, triangulatePoint3Vector)
{
  gtsam::Pose3 X_world_camera_prev;
  // Use Quaternion and translation to construct the current pose
  // Around y axis, 15 degree - equivalent to 15 degree to the right in yaw
  gtsam::Pose3 X_world_camera_curr(gtsam::Rot3(0.9914449, 0.0, 0.1305262, 0.0), gtsam::Point3(0.1, 0.1, 2.0));

  // std::cout << X_world_camera_prev << "\n";
  // std::cout << X_world_camera_curr << "\n";

  gtsam::Matrix3 obj_rot = Eigen::Matrix3d::Identity();
  // Euler angle x = 5 degree, y = 10 degree, z = 15 degree, order XYZ
  obj_rot <<  0.98106026, -0.08583165,  0.17364818,
              0.12895841,  0.95833311, -0.254887,
             -0.14453543,  0.2724529,   0.95125124;

  // std::cout << obj_rot << "\n";

  gtsam::Matrix3 intrinsic = Eigen::Matrix3d::Identity();
  intrinsic(0, 0) = 320.0;
  intrinsic(1, 1) = 320.0;
  intrinsic(0, 2) = 320.0;
  intrinsic(1, 2) = 240.0;

  // std::cout << intrinsic << "\n";

  gtsam::Point3Vector points_prev, points_curr;

  // a 2*2*2 cude
  points_prev.push_back(Eigen::Vector3d(5.0, 1.0, 5.0));
  points_prev.push_back(Eigen::Vector3d(7.0, 1.0, 5.0));
  points_prev.push_back(Eigen::Vector3d(5.0, 1.0, 7.0));
  points_prev.push_back(Eigen::Vector3d(7.0, 1.0, 7.0));
  points_prev.push_back(Eigen::Vector3d(5.0, -1.0, 5.0));
  points_prev.push_back(Eigen::Vector3d(7.0, -1.0, 5.0));
  points_prev.push_back(Eigen::Vector3d(5.0, -1.0, 7.0));
  points_prev.push_back(Eigen::Vector3d(7.0, -1.0, 7.0));

  // same rotation as obj_rot
  gtsam::Pose3 H_prev_curr_world(gtsam::Rot3(0.9862359, 0.1336749, 0.0806561, 0.0544469), gtsam::Point3(2.0, 0.0, 0.0));
  // std::cout << "H_prev_curr_world\n" << H_prev_curr_world << std::endl;
  for (int i = 0; i < 8; i++){
    points_curr.push_back(H_prev_curr_world.transformFrom(points_prev[i]));
  }


  gtsam::Point2Vector observation_prev, observation_curr;

  for (int i = 0; i < 8; i++){
    gtsam::Point3 local_point_prev = intrinsic*X_world_camera_prev.transformTo(points_prev[i]);
    gtsam::Point3 local_point_curr = intrinsic*X_world_camera_curr.transformTo(points_curr[i]);

    observation_prev.push_back(Eigen::Vector2d(local_point_prev.x()/local_point_prev.z(), local_point_prev.y()/local_point_prev.z()));
    observation_curr.push_back(Eigen::Vector2d(local_point_curr.x()/local_point_curr.z(), local_point_curr.y()/local_point_curr.z()));
  }


  gtsam::Point3Vector points_world = dyno::mono_backend_tools::triangulatePoint3Vector(X_world_camera_prev, X_world_camera_curr, intrinsic,
                                                                                       observation_prev, observation_curr, obj_rot);

  gtsam::Point3Vector expected_points_world;
  // [ 3.07637456  4.69454556  2.5236143   3.96822456  3.30434543  5.0268543   2.68667343  4.19481659]
  // [ 0.61527491  0.67064937  0.50472286  0.56688922 -0.66086909 -0.71812204 -0.53733469 -0.59925951]
  // [ 3.07637456  3.35324683  3.53306001  3.96822456  3.30434543  3.59061022  3.7613428   4.19481659]
  expected_points_world.push_back(gtsam::Point3(3.07637456, 0.61527491, 3.07637456));
  expected_points_world.push_back(gtsam::Point3(4.69454556, 0.67064937, 3.35324683));
  expected_points_world.push_back(gtsam::Point3(2.5236143 , 0.50472286 , 3.53306001));
  expected_points_world.push_back(gtsam::Point3(3.96822456, 0.56688922, 3.96822456));
  expected_points_world.push_back(gtsam::Point3(3.30434543, -0.66086909, 3.30434543));
  expected_points_world.push_back(gtsam::Point3( 5.0268543, -0.71812204, 3.59061022));
  expected_points_world.push_back(gtsam::Point3(2.68667343, -0.53733469, 3.7613428));
  expected_points_world.push_back(gtsam::Point3(4.19481659, -0.59925951,4.19481659));

  EXPECT_EQ(expected_points_world.size(), points_world.size());
  for(size_t i = 0; i < expected_points_world.size(); i++) {
    EXPECT_TRUE(gtsam::assert_equal(expected_points_world.at(i), points_world.at(i), 1.0e-5));
  }
}

TEST(MonoBackendTools, triangulatePoint3VectorNonExpanded)
{
  gtsam::Pose3 X_world_camera_prev;
  // Use Quaternion and translation to construct the current pose
  // Around y axis, 15 degree - equivalent to 15 degree to the right in yaw
  gtsam::Pose3 X_world_camera_curr(gtsam::Rot3(0.9914449, 0.0, 0.1305262, 0.0), gtsam::Point3(0.1, 0.1, 2.0));

  // std::cout << X_world_camera_prev << "\n";
  // std::cout << X_world_camera_curr << "\n";

  gtsam::Matrix3 obj_rot = Eigen::Matrix3d::Identity();
  // Euler angle x = 5 degree, y = 10 degree, z = 15 degree, order XYZ
  obj_rot <<  0.98106026, -0.08583165,  0.17364818,
              0.12895841,  0.95833311, -0.254887,
             -0.14453543,  0.2724529,   0.95125124;

  // std::cout << obj_rot << "\n";

  gtsam::Matrix3 intrinsic = Eigen::Matrix3d::Identity();
  intrinsic(0, 0) = 320.0;
  intrinsic(1, 1) = 320.0;
  intrinsic(0, 2) = 320.0;
  intrinsic(1, 2) = 240.0;

  // std::cout << intrinsic << "\n";

  gtsam::Point3Vector points_prev, points_curr;

  // a 2*2*2 cude
  points_prev.push_back(Eigen::Vector3d(5.0, 1.0, 5.0));
  points_prev.push_back(Eigen::Vector3d(7.0, 1.0, 5.0));
  points_prev.push_back(Eigen::Vector3d(5.0, 1.0, 7.0));
  points_prev.push_back(Eigen::Vector3d(7.0, 1.0, 7.0));
  points_prev.push_back(Eigen::Vector3d(5.0, -1.0, 5.0));
  points_prev.push_back(Eigen::Vector3d(7.0, -1.0, 5.0));
  points_prev.push_back(Eigen::Vector3d(5.0, -1.0, 7.0));
  points_prev.push_back(Eigen::Vector3d(7.0, -1.0, 7.0));

  // same rotation as obj_rot
  gtsam::Pose3 H_prev_curr_world(gtsam::Rot3(0.9862359, 0.1336749, 0.0806561, 0.0544469), gtsam::Point3(2.0, 0.0, 0.0));
  // std::cout << "H_prev_curr_world\n" << H_prev_curr_world << std::endl;
  for (int i = 0; i < 8; i++){
    points_curr.push_back(H_prev_curr_world.transformFrom(points_prev[i]));
  }


  gtsam::Point2Vector observation_prev, observation_curr;

  for (int i = 0; i < 8; i++){
    gtsam::Point3 local_point_prev = intrinsic*X_world_camera_prev.transformTo(points_prev[i]);
    gtsam::Point3 local_point_curr = intrinsic*X_world_camera_curr.transformTo(points_curr[i]);

    observation_prev.push_back(Eigen::Vector2d(local_point_prev.x()/local_point_prev.z(), local_point_prev.y()/local_point_prev.z()));
    observation_curr.push_back(Eigen::Vector2d(local_point_curr.x()/local_point_curr.z(), local_point_curr.y()/local_point_curr.z()));
  }


  gtsam::Point3Vector points_world = dyno::mono_backend_tools::triangulatePoint3VectorNonExpanded(X_world_camera_prev, X_world_camera_curr, intrinsic,
                                                                                                  observation_prev, observation_curr, obj_rot);

  gtsam::Point3Vector expected_points_world;
  // [ 3.81828248  5.7540693   3.24108742  4.98904018  4.00304244  6.02650317  3.38370898  5.18169805]
  // [ 0.7636565   0.8220099   0.64821748  0.71272003 -0.80060849 -0.86092902 -0.6767418  -0.74024258]
  // [ 3.81828248  4.1100495   4.53752238  4.98904018  4.00304244  4.30464512  4.73719257  5.18169805]
  expected_points_world.push_back(gtsam::Point3(3.81828248, 0.7636565, 3.81828248));
  expected_points_world.push_back(gtsam::Point3(5.7540693,  0.8220099, 4.1100495));
  expected_points_world.push_back(gtsam::Point3(3.24108742, 0.64821748, 4.53752238));
  expected_points_world.push_back(gtsam::Point3(4.98904018, 0.71272003, 4.98904018));
  expected_points_world.push_back(gtsam::Point3(4.00304244, -0.80060849, 4.00304244));
  expected_points_world.push_back(gtsam::Point3(6.02650317, -0.86092902, 4.30464512));
  expected_points_world.push_back(gtsam::Point3(3.38370898, -0.6767418, 4.73719257));
  expected_points_world.push_back(gtsam::Point3(5.18169805, -0.74024258, 5.18169805));

  EXPECT_EQ(expected_points_world.size(), points_world.size());
  for(size_t i = 0; i < expected_points_world.size(); i++) {
    EXPECT_TRUE(gtsam::assert_equal(expected_points_world.at(i), points_world.at(i), 1.0e-5));
  }
}
```

## File: camera/ZEDCamera/test_ZED_camera_init.cc
```
// test_zed_camera.cc

#include <gtest/gtest.h>
#include <glog/logging.h>
#include <sl/Camera.hpp> // ZED SDK header

#include "dynosam/common/ZEDCamera.hpp" // Header for the class under test


namespace dyno {
namespace common {
namespace test {


TEST(ZEDCameraInitTest, ZC_INIT_001_LiveCameraConfigPopulation) {
    // **Test Case ID:** ZC_INIT_001
    // **Method(s) Under Test:** ZEDCamera(const ZEDCameraConfig& config), populateInitParams()
    // **Test Scenario Description:** Verify correct population of sl::InitParameters for a live camera.
    // **Key Configurations/Inputs:** Live mode, specific resolution, FPS, depth mode, IMU enabled.
    // **Expected Outcome:** zed_init_params_ member reflects the input config.
    // **Mocking Considerations:** None for this part of initialization.

    ZEDCameraConfig config;
    config.svo_file_path = ""; // Live camera
    config.resolution = sl::RESOLUTION::HD720;
    config.fps = 30;
    config.depth_mode = sl::DEPTH_MODE::PERFORMANCE;
    config.coordinate_units = sl::UNIT::METER;
    config.coordinate_system_3d = sl::COORDINATE_SYSTEM::RIGHT_HANDED_Z_UP_X_FWD;
    config.enable_imu = true;
    // Set other defaults or specific values as needed by ZEDCameraConfig
    config.sdk_verbose = 1;
    config.camera_disable_self_calib = false;
    config.enable_image_enhancement = true;
    config.open_timeout_sec = 5.0f;
    config.enable_right_side_measure = false;
    config.async_grab_camera_recovery = true;
    config.grab_compute_capping_fps = 0; // 0 for no capping based on grab
    config.enable_image_validity_check = true;


    ZEDCamera zed_cam(config);
    EXPECT_TRUE(zed_cam.open()); // Ensure camera opens successfully

    // Access public member zed_init_params_ for verification
    const sl::InitParameters& init_params = zed_cam.getCameraInitParams();

    EXPECT_EQ(init_params.camera_resolution, config.resolution);
    EXPECT_EQ(init_params.camera_fps, config.fps);
    EXPECT_EQ(init_params.depth_mode, config.depth_mode);
    EXPECT_EQ(init_params.coordinate_units, config.coordinate_units);
    EXPECT_EQ(init_params.coordinate_system, config.coordinate_system_3d);
    EXPECT_TRUE(init_params.sensors_required); // Because config.enable_imu = true

    EXPECT_EQ(init_params.sdk_verbose, config.sdk_verbose > 0);
    EXPECT_EQ(init_params.camera_disable_self_calib, config.camera_disable_self_calib);
    EXPECT_EQ(init_params.enable_image_enhancement, config.enable_image_enhancement);
    EXPECT_FLOAT_EQ(init_params.open_timeout_sec, config.open_timeout_sec);
    EXPECT_EQ(init_params.enable_right_side_measure, config.enable_right_side_measure);
    EXPECT_EQ(init_params.async_grab_camera_recovery, config.async_grab_camera_recovery);
    EXPECT_EQ(init_params.grab_compute_capping_fps, config.grab_compute_capping_fps);
    EXPECT_EQ(init_params.enable_image_validity_check, config.enable_image_validity_check);

    // For live camera, input type should not be SVO
    EXPECT_NE(static_cast<const sl::INPUT_TYPE>(init_params.input.getType()), sl::INPUT_TYPE::SVO);
}

TEST(ZEDCameraInitTest, ZC_INIT_002_SVOFileConfigPopulation) {
    // **Test Case ID:** ZC_INIT_002
    // **Method(s) Under Test:** ZEDCamera(const ZEDCameraConfig& config), populateInitParams()
    // **Test Scenario Description:** Verify correct population of sl::InitParameters for SVO.
    // **Key Configurations/Inputs:** SVO file path, svo_real_time_mode = true, IMU disabled.
    // **Expected Outcome:** zed_init_params_.input reflects SVO, sensors_required is false.
    // **Mocking Considerations:** None for this part of initialization.

    ZEDCameraConfig config;
    config.svo_file_path = "test_dummy.svo";
    config.svo_real_time_mode = true;
    config.enable_imu = false;
    // Set other defaults or specific values
    config.resolution = sl::RESOLUTION::VGA;
    config.fps = 15;
    config.depth_mode = sl::DEPTH_MODE::ULTRA;
    config.coordinate_units = sl::UNIT::MILLIMETER;
    config.coordinate_system_3d = sl::COORDINATE_SYSTEM::IMAGE;


    ZEDCamera zed_cam(config);

    const sl::InitParameters& init_params = zed_cam.getCameraInitParams();

    EXPECT_EQ(static_cast<sl::INPUT_TYPE>(init_params.input.getType()), sl::INPUT_TYPE::SVO);
    // We can't directly get the path string from sl::InputType easily without SDK interaction
    // but we trust setFromSVOFile worked. The important part is the type and svo_real_time_mode.
    EXPECT_TRUE(init_params.svo_real_time_mode);
    EXPECT_FALSE(init_params.sensors_required); // Because config.enable_imu = false

    EXPECT_EQ(init_params.camera_resolution, config.resolution);
    EXPECT_EQ(init_params.camera_fps, config.fps);
    EXPECT_EQ(init_params.depth_mode, config.depth_mode);
    EXPECT_EQ(init_params.coordinate_units, config.coordinate_units);
    EXPECT_EQ(init_params.coordinate_system, config.coordinate_system_3d);
}

TEST(ZEDCameraInitTest, ZC_INIT_003_BooleanFlagsPopulation) {
    // **Test Case ID:** ZC_INIT_003
    // **Method(s) Under Test:** ZEDCamera(const ZEDCameraConfig& config), populateInitParams()
    // **Test Scenario Description:** Verify populateInitParams handles various boolean flags correctly.
    // **Key Configurations/Inputs:** Specific boolean flags set to non-default values.
    // **Expected Outcome:** Corresponding fields in zed_init_params_ match the config.
    // **Mocking Considerations:** None for this part of initialization.

    ZEDCameraConfig config;
    // Default some non-boolean values to ensure the test focuses on booleans
    config.svo_file_path = "";
    config.resolution = sl::RESOLUTION::HD1080;
    config.fps = 60;

    // Set boolean flags to specific values
    config.camera_disable_self_calib = true;
    config.enable_image_enhancement = false;
    config.async_grab_camera_recovery = false;
    config.enable_right_side_measure = true;
    config.grab_compute_capping_fps = 15;
    config.open_timeout_sec = 3.5f;
    config.enable_image_validity_check = false; // Test this specific one

    ZEDCamera zed_cam(config);

    const sl::InitParameters& init_params = zed_cam.getCameraInitParams();

    EXPECT_EQ(init_params.camera_disable_self_calib, config.camera_disable_self_calib);
    EXPECT_EQ(init_params.enable_image_enhancement, config.enable_image_enhancement);
    EXPECT_EQ(init_params.async_grab_camera_recovery, config.async_grab_camera_recovery);
    EXPECT_EQ(init_params.enable_right_side_measure, config.enable_right_side_measure);
    EXPECT_EQ(init_params.grab_compute_capping_fps, config.grab_compute_capping_fps);
    EXPECT_FLOAT_EQ(init_params.open_timeout_sec, config.open_timeout_sec);
    EXPECT_EQ(init_params.enable_image_validity_check, config.enable_image_validity_check);
}


} // namespace test
} // namespace common
} // namespace dyno
// int main(int argc, char** argv) {
//     ::testing::InitGoogleTest(&argc, argv);
//     google::InitGoogleLogging(argv[0]);
//     return RUN_ALL_TESTS();
// }
```

## File: camera/test_camera_params.cc
```
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

#include "dynosam/common/CameraParams.hpp"
#include "internal/helpers.hpp"



#include <config_utilities/parsing/yaml.h>

#include <gtsam/geometry/Cal3DS2.h>

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <gtest/gtest.h>

#include <thread>

DECLARE_string(test_data_path);

using namespace dyno;

TEST(testCameraParamss, basicConstructionCal3DS2)
{
  // Intrinsics.
  const std::vector<double> intrinsics_expected = { 458.654, 457.296, 367.215, 248.375 };
  //   // Distortion coefficients.
  const std::vector<double> distortion_expected = { -0.28340811, 0.07395907, 0.00019359, 1.76187114e-05 };

  const cv::Size size_expected(752, 480);

  const std::string expected_distortion_model = "radtan";

  // Sensor extrinsics wrt. the body-frame.
  gtsam::Rot3 R_expected(0.0148655429818, -0.999880929698, 0.00414029679422, 0.999557249008, 0.0149672133247,
                         0.025715529948, -0.0257744366974, 0.00375618835797, 0.999660727178);
  gtsam::Point3 T_expected(-0.0216401454975, -0.064676986768, 0.00981073058949);
  gtsam::Pose3 pose_expected(R_expected, T_expected);

  CameraParams params(intrinsics_expected, distortion_expected, size_expected, expected_distortion_model,
                     pose_expected);

  EXPECT_EQ(size_expected.width, params.ImageWidth());
  EXPECT_EQ(size_expected.height, params.ImageHeight());

  // for (int c = 0u; c < 4u; c++)
  // {
  //   EXPECT_DOUBLE_EQ(intrinsics_expected[c], params.intrinsics_[c]);
  // }
  EXPECT_DOUBLE_EQ(intrinsics_expected[0], params.getCameraMatrix().at<double>(0, 0));
  EXPECT_DOUBLE_EQ(intrinsics_expected[1], params.getCameraMatrix().at<double>(1, 1));
  EXPECT_DOUBLE_EQ(intrinsics_expected[2], params.getCameraMatrix().at<double>(0, 2));
  EXPECT_DOUBLE_EQ(intrinsics_expected[3], params.getCameraMatrix().at<double>(1, 2));

  EXPECT_DOUBLE_EQ(intrinsics_expected[0], params.fx());
  EXPECT_DOUBLE_EQ(intrinsics_expected[1], params.fy());
  EXPECT_DOUBLE_EQ(intrinsics_expected[2], params.cu());
  EXPECT_DOUBLE_EQ(intrinsics_expected[3], params.cv());
  // //   EXPECT_EQ(cam_params.intrinsics_.size(), 4u);
  gtsam::Cal3DS2 gtsam_calib = params.constructGtsamCalibration<gtsam::Cal3DS2>();

  EXPECT_DOUBLE_EQ(intrinsics_expected[0], gtsam_calib.fx());
  EXPECT_DOUBLE_EQ(intrinsics_expected[1], gtsam_calib.fy());
  EXPECT_DOUBLE_EQ(0u, gtsam_calib.skew());
  EXPECT_DOUBLE_EQ(intrinsics_expected[2], gtsam_calib.px());
  EXPECT_DOUBLE_EQ(intrinsics_expected[3], gtsam_calib.py());

  EXPECT_TRUE(assert_equal(pose_expected, params.getExtrinsics()));

  for (int c = 0u; c < 4u; c++)
  {
    EXPECT_DOUBLE_EQ(distortion_expected[c], params.getDistortionCoeffs().at<double>(c));
  }
  EXPECT_EQ(params.getDistortionCoeffs().rows, 1u);
  EXPECT_EQ(params.getDistortionCoeffs().cols, 4u);
  EXPECT_DOUBLE_EQ(distortion_expected[0], gtsam_calib.k1());
  EXPECT_DOUBLE_EQ(distortion_expected[1], gtsam_calib.k2());
  EXPECT_DOUBLE_EQ(distortion_expected[2], gtsam_calib.p1());
  EXPECT_DOUBLE_EQ(distortion_expected[3], gtsam_calib.p2());
}


TEST(testCameraParams, parseYAML) {
  // CameraParams cam_params = CameraParams::fromYamlFile(getTestDataPath() + "/sensor.yaml");
  auto cam_params = config::fromYamlFile<CameraParams>(getTestDataPath() + "/sensor.yaml");

  // Frame rate.
  const double frame_rate_expected = 1.0 / 20.0;
  // EXPECT_DOUBLE_EQ(frame_rate_expected, cam_params.frame_rate_);

  // Image size.
  const cv::Size size_expected(752, 480);
  EXPECT_EQ(size_expected.width, cam_params.ImageWidth());
  EXPECT_EQ(size_expected.height, cam_params.ImageHeight());

  // Intrinsics.
  const std::vector<double> intrinsics_expected = {
      458.654, 457.296, 367.215, 248.375};
  EXPECT_DOUBLE_EQ(intrinsics_expected[0], cam_params.fx());
  EXPECT_DOUBLE_EQ(intrinsics_expected[1], cam_params.fy());
  EXPECT_DOUBLE_EQ(intrinsics_expected[2], cam_params.cu());
  EXPECT_DOUBLE_EQ(intrinsics_expected[3], cam_params.cv());

  EXPECT_DOUBLE_EQ(intrinsics_expected[0], cam_params.getCameraMatrix().at<double>(0, 0));
  EXPECT_DOUBLE_EQ(intrinsics_expected[1], cam_params.getCameraMatrix().at<double>(1, 1));
  EXPECT_DOUBLE_EQ(intrinsics_expected[2], cam_params.getCameraMatrix().at<double>(0, 2));
  EXPECT_DOUBLE_EQ(intrinsics_expected[3], cam_params.getCameraMatrix().at<double>(1, 2));
  gtsam::Cal3DS2 gtsam_calib = cam_params.constructGtsamCalibration<gtsam::Cal3DS2>();

  EXPECT_DOUBLE_EQ(intrinsics_expected[0], gtsam_calib.fx());
  EXPECT_DOUBLE_EQ(intrinsics_expected[1], gtsam_calib.fy());
  EXPECT_DOUBLE_EQ(0u, gtsam_calib.skew());
  EXPECT_DOUBLE_EQ(intrinsics_expected[2], gtsam_calib.px());
  EXPECT_DOUBLE_EQ(intrinsics_expected[3], gtsam_calib.py());

  // Sensor extrinsics wrt. the body-frame.
  gtsam::Rot3 R_expected(0.0148655429818,
                         -0.999880929698,
                         0.00414029679422,
                         0.999557249008,
                         0.0149672133247,
                         0.025715529948,
                         -0.0257744366974,
                         0.00375618835797,
                         0.999660727178);
  gtsam::Point3 T_expected(-0.0216401454975, -0.064676986768, 0.00981073058949);
  gtsam::Pose3 pose_expected(R_expected, T_expected);
  EXPECT_TRUE(assert_equal(pose_expected, cam_params.getExtrinsics()));

  // Distortion coefficients.
  const std::vector<double> distortion_expected = {
      -0.28340811, 0.07395907, 0.00019359, 1.76187114e-05};
  for (size_t c = 0u; c < 4u; c++) {
    EXPECT_DOUBLE_EQ(distortion_expected[c],
                     cam_params.getDistortionCoeffs().at<double>(c));
  }
  EXPECT_EQ(cam_params.getDistortionCoeffs().rows, 1u);
  EXPECT_EQ(cam_params.getDistortionCoeffs().cols, 4u);
  EXPECT_DOUBLE_EQ(distortion_expected[0], gtsam_calib.k1());
  EXPECT_DOUBLE_EQ(distortion_expected[1], gtsam_calib.k2());
  EXPECT_DOUBLE_EQ(distortion_expected[2], gtsam_calib.p1());
  EXPECT_DOUBLE_EQ(distortion_expected[3], gtsam_calib.p2());
}

TEST(testCameraParamss, convertDistortionVectorToMatrix)
{
  std::vector<double> distortion_coeffs;

  // 4 distortion params
  distortion_coeffs = { 1.0, -2.0, 1.3, 10 };
  cv::Mat distortion_coeffs_mat;
  CameraParams::convertDistortionVectorToMatrix(distortion_coeffs, &distortion_coeffs_mat);
  EXPECT_EQ(distortion_coeffs_mat.cols, distortion_coeffs.size());
  EXPECT_EQ(distortion_coeffs_mat.rows, 1u);
  for (size_t i = 0u; i < distortion_coeffs.size(); i++)
  {
    EXPECT_EQ(distortion_coeffs_mat.at<double>(0, i), distortion_coeffs.at(i));
  }

  // 5 distortion params
  distortion_coeffs = { 1, 1.2f, 3u, 4l, 5.34 };  //! randomize types as well
  CameraParams::convertDistortionVectorToMatrix(distortion_coeffs, &distortion_coeffs_mat);
  EXPECT_EQ(distortion_coeffs_mat.cols, distortion_coeffs.size());
  EXPECT_EQ(distortion_coeffs_mat.rows, 1u);
  for (size_t i = 0u; i < distortion_coeffs.size(); i++)
  {
    EXPECT_EQ(distortion_coeffs_mat.at<double>(0u, i), distortion_coeffs.at(i));
  }

  // n distortion params
  distortion_coeffs = { 1.0, 1.2, 3.2, 4.3, 5.34, 10203, 1818.9, 1.9 };
  CameraParams::convertDistortionVectorToMatrix(distortion_coeffs, &distortion_coeffs_mat);
  EXPECT_EQ(distortion_coeffs_mat.cols, distortion_coeffs.size());
  EXPECT_EQ(distortion_coeffs_mat.rows, 1u);
  for (size_t i = 0u; i < distortion_coeffs.size(); i++)
  {
    EXPECT_EQ(distortion_coeffs_mat.at<double>(0u, i), distortion_coeffs.at(i));
  }
}
```

## File: camera/test_camera.cc
```
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

#include "internal/helpers.hpp"


#include <gflags/gflags.h>
#include <glog/logging.h>
#include <gtest/gtest.h>

#include "dynosam/common/Camera.hpp"
#include "dynosam/utils/Numerical.hpp"

#include <gtsam/base/numericalDerivative.h>

using namespace dyno;

TEST(Camera, project)
{
  Landmarks lmks;
  lmks.push_back(Landmark(0.0, 0.0, 1.0));
  lmks.push_back(Landmark(0.0, 0.0, 2.0));
  lmks.push_back(Landmark(0.0, 1.0, 2.0));
  lmks.push_back(Landmark(0.0, 10.0, 20.0));
  lmks.push_back(Landmark(1.0, 0.0, 2.0));

  CameraParams::IntrinsicsCoeffs intrinsics(4);
  CameraParams::DistortionCoeffs distortion(4);

  intrinsics.at(0) = 1.0;  // fx
  intrinsics.at(1) = 1.0;  // fy
  intrinsics.at(2) = 3.0;  // u0
  intrinsics.at(3) = 2.0;  // v0
  Keypoints expected_kpts;
  expected_kpts.push_back(Keypoint(intrinsics.at(2), intrinsics.at(3)));
  expected_kpts.push_back(Keypoint(intrinsics.at(2), intrinsics.at(3)));
  expected_kpts.push_back(Keypoint(3.0, 1.0 / 2.0 + 2.0));
  expected_kpts.push_back(Keypoint(3.0, 1.0 / 2.0 + 2.0));
  expected_kpts.push_back(Keypoint(1.0 / 2.0 + 3.0, 2.0));

  CameraParams camera_params(intrinsics, distortion, cv::Size(640, 480), "radtan");

  Camera camera(camera_params);

  Keypoints actual_kpts;
  EXPECT_NO_THROW(camera.project(lmks, &actual_kpts));
  dyno_testing::compareKeypoints(actual_kpts, expected_kpts);
}

TEST(Camera, backProjectSingleSimple)
{
  // Easy test first, back-project keypoint at the center of the image with
  // a given depth.
  CameraParams camera_params = dyno_testing::makeDefaultCameraParams();
  Camera camera(camera_params);

  Keypoint kpt(camera_params.cu(), camera_params.cv());
  Landmark actual_lmk;
  double depth = 2.0;
  camera.backProject(kpt, depth, &actual_lmk);

  Landmark expected_lmk(0.0, 0.0, depth);
  EXPECT_NEAR(expected_lmk.x(), actual_lmk.x(), 0.0001);
  EXPECT_NEAR(expected_lmk.y(), actual_lmk.y(), 0.0001);
  EXPECT_NEAR(expected_lmk.z(), actual_lmk.z(), 0.0001);
}

TEST(Camera, backProjectMultipleSimple)
{
  // Easy test first, back-project keypoints at the center of the image with
  // different depths.
  CameraParams camera_params = dyno_testing::makeDefaultCameraParams();
  Camera camera(camera_params);

  Keypoint kpt(camera_params.cu(), camera_params.cv());
  // Create 3 keypoints centered at image with different depths
  Keypoints kpts(3, kpt);
  Depths depths = { 2.0, 3.0, 4.5 };
  Landmarks actual_lmks;
  camera.backProject(kpts, depths, &actual_lmks);

  Landmarks expected_lmks;
  for (const auto& depth : depths)
  {
    expected_lmks.push_back(Landmark(0.0, 0.0, depth));
  }

  dyno_testing::compareLandmarks(actual_lmks, expected_lmks);
}

TEST(Camera, backProjectSingleTopLeft)
{
  // Back-project keypoint at the center of the image with a given depth.
  CameraParams::IntrinsicsCoeffs intrinsics(4);
  CameraParams::DistortionCoeffs distortion(4);

  double fx = 30.9 / 2.2;
  double fy = 12.0 / 23.0;
  double cu = 390.8;
  double cv = 142.2;

  intrinsics.at(0) = fx;  // fx
  intrinsics.at(1) = fy;  // fy
  intrinsics.at(2) = cu;  // u0
  intrinsics.at(3) = cv;  // v0
  CameraParams camera_params(intrinsics, distortion, cv::Size(640, 480), "radtan");

  Camera camera(camera_params);

  Landmark actual_lmk;
  double depth = 2.0;
  Keypoint kpt(0.0, 0.0);  // Top-left corner
  camera.backProject(kpt, depth, &actual_lmk);

  Landmark expected_lmk(depth / fx * (-cu), depth / fy * (-cv), depth);
  EXPECT_NEAR(expected_lmk.x(), actual_lmk.x(), 0.0001);
  EXPECT_NEAR(expected_lmk.y(), actual_lmk.y(), 0.0001);
  EXPECT_NEAR(expected_lmk.z(), actual_lmk.z(), 0.0001);
}

TEST(Camera, backProjectToZ)
{
  CameraParams camera_params = dyno_testing::makeDefaultCameraParams();
  Camera camera(camera_params);

  Keypoint kpt(camera_params.cu()/2.0, camera_params.cv()/2.0);
  double depth = 2.0;

  Landmark actual_lmk;
  camera.backProject(kpt, depth, &actual_lmk);

  const double Z = actual_lmk(2);
  Landmark z_projected_lmk;
  camera.backProjectFromZ(kpt, Z, &z_projected_lmk);

  //check that the keypoint is the same as the actual one (we just change the Z)
  //but the "measurement" shold remain the same
  Keypoint calculated_kp;
  camera.project(z_projected_lmk, &calculated_kp);
  EXPECT_TRUE(gtsam::assert_equal(kpt, calculated_kp));


}


TEST(Camera, backProjectToZJacobian)
{
  CameraParams camera_params = dyno_testing::makeDefaultCameraParams();
  Camera camera(camera_params);

  Keypoint kpt(camera_params.cu()/2.0, camera_params.cv()/2.0);
  double depth = 2.0;

  Landmark actual_lmk;
  camera.backProject(kpt, depth, &actual_lmk);
  const double Z = actual_lmk(2);


  gtsam::Pose3 pose(gtsam::Rot3::Rodrigues(0,2,3),gtsam::Point3(1,2,0));


  auto numerical_deriv_func =[&camera](const gtsam::Vector3& uvz, const gtsam::Pose3& X_world) -> gtsam::Vector3 {
    Landmark lmk;
    camera.backProjectFromZ(gtsam::Point2(uvz(0), uvz(1)), uvz(2), &lmk, X_world);
    return lmk;
  };


  //construct 3x1 vector of input to satisfy the matrix structure of the problem we want to sove
  gtsam::Vector3 input(kpt(0), kpt(1), Z);
  //numericalDerivative21 -> 2 function arguments, derivative w.r.t uvz
  gtsam::Matrix33 numerical_J = gtsam::numericalDerivative21<gtsam::Vector3, gtsam::Vector3, const gtsam::Pose3&>(
    numerical_deriv_func, input, pose);

  Landmark lmk; //unused
  gtsam::Matrix33 analytical_J;
  camera.backProjectFromZ(kpt, Z, &lmk, pose, analytical_J);

  EXPECT_TRUE(gtsam::assert_equal(numerical_J, analytical_J));

}
```

## File: core_utils/test_code_concepts.cc
```
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

#include <glog/logging.h>
#include <gtest/gtest.h>
#include <exception>
#include <opencv4/opencv2/opencv.hpp>

#include <type_traits>

#include "dynosam/visualizer/ColourMap.hpp"
#include "dynosam/utils/OpenCVUtils.hpp"
#include "dynosam/logger/Logger.hpp"
#include "dynosam/backend/FactorGraphTools.hpp"
#include "dynosam/backend/BackendDefinitions.hpp"

#include <optional>
#include <atomic>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Cal3_S2Stereo.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/utilities.h>
#include <gtsam/nonlinear/NonlinearEquality.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/StereoFactor.h>
#include <gtsam/slam/dataset.h>

#include <gtsam_unstable/slam/PoseToPointFactor.h>
#include <gtsam/base/treeTraversal-inst.h>

#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/base/Matrix.h>

#include <eigen3/unsupported/Eigen/MatrixFunctions>
#include <eigen3/unsupported/Eigen/KroneckerProduct>


#include <string>
#include <fstream>
#include <iostream>


std::vector<gtsam::Point3> createPoints() {

  // Create the set of ground-truth landmarks
  std::vector<gtsam::Point3> points;
  points.push_back(gtsam::Point3(10.0,10.0,10.0));
  points.push_back(gtsam::Point3(-10.0,10.0,10.0));
  points.push_back(gtsam::Point3(-10.0,-10.0,10.0));
  points.push_back(gtsam::Point3(10.0,-10.0,10.0));
  points.push_back(gtsam::Point3(10.0,10.0,-10.0));
  points.push_back(gtsam::Point3(-10.0,10.0,-10.0));
  points.push_back(gtsam::Point3(-10.0,-10.0,-10.0));
  points.push_back(gtsam::Point3(10.0,-10.0,-10.0));

  return points;
}

/* ************************************************************************* */
std::vector<gtsam::Pose3> createPoses(
            const gtsam::Pose3& init = gtsam::Pose3(gtsam::Rot3::Ypr(M_PI/2,0,-M_PI/2), gtsam::Point3(30, 0, 0)),
            const gtsam::Pose3& delta = gtsam::Pose3(gtsam::Rot3::Ypr(0,-M_PI/4,0), gtsam::Point3(sin(M_PI/4)*30, 0, 30*(1-sin(M_PI/4)))),
            int steps = 8) {

  // Create the set of ground-truth poses
  // Default values give a circular trajectory, radius 30 at pi/4 intervals, always facing the circle center
  std::vector<gtsam::Pose3> poses;
  int i = 1;
  poses.push_back(init);
  for(; i < steps; ++i) {
    poses.push_back(poses[i-1].compose(delta));
  }

  return poses;
}

cv::Mat GetSquareImage( const cv::Mat& img, int target_width = 500 )
{
    int width = img.cols,
       height = img.rows;

    cv::Mat square = cv::Mat::zeros( target_width, target_width, img.type() );

    int max_dim = ( width >= height ) ? width : height;
    float scale = ( ( float ) target_width ) / max_dim;
    cv::Rect roi;
    if ( width >= height )
    {
        roi.width = target_width;
        roi.x = 0;
        roi.height = height * scale;
        roi.y = ( target_width - roi.height ) / 2;
    }
    else
    {
        roi.y = 0;
        roi.height = target_width;
        roi.width = width * scale;
        roi.x = ( target_width - roi.width ) / 2;
    }

    cv::resize( img, square( roi ), roi.size() );

    return square;
}

//we need compose, inverse (and thats it?)

template<typename T>
struct indexed_transform_traits;

//actually just want this for Pose operations, i guess? Do I both generalising?
// this should happily cover all Eigen stuff as well :)
template<typename T, class = typename  std::enable_if<is_gtsam_value_v<T>>>
struct GtsamIndexedTransformTraits {

    static T Inverse(const T& t) {
        return gtsam::traits<T>::Inverse(t);
    }

    static T Compose(const T& g, const T& h) {
        return gtsam::traits<T>::Compose(g, h);
    }


};
template<typename T>
struct indexed_transform_traits : GtsamIndexedTransformTraits<T> {};


/**
 * @brief Pose class that add the 3-indexed notation from the "Pose changes
 * from a different point of view" paper
 *
 */
template<typename T>
class IndexedTransform {
public:
  using This = IndexedTransform<T>;

  // Top left index representing observing frame
  std::string tl;
  // Bottom left index, representing from frame
  std::string bl{};
  // Bottom right index, representing to frame
  std::string br{};
  // Data
  T transform;

  IndexedTransform(
    const std::string& top_left,
    const std::string& bottom_left,
    const std::string& bottom_right,
    const T& t)
    :   tl(top_left), bl(bottom_left), br(bottom_right), transform(t) {}

  const std::string& Origin() const { return tl; }
  const std::string& From() const { return bl; }
  const std::string& To() const { return br; }

  /**
   * @brief Returns true if the captured data represents a pose or a motion.
   * As per Equation 10 and 11 we can, if the top left and bottom left notation is the same,
   * it can be re-written in the ususal two index form
   *
   * @return true
   * @return false
   */
  inline bool isCoordinateTransform() const {
    return tl == bl;
  }

  operator T() { return transform; }

  This compose(const This& t) const {
    // T new_transform = transform * t.transform;
    T new_transform = indexed_transform_traits<T>::Compose(transform, t.transform);

    //if this is relative (left hand side), both must be relative
    if(isCoordinateTransform()) {
        if (!t.isCoordinateTransform()) {
            throw std::runtime_error("");
        }
        else {
            //check that the coordinates match
            if(br != t.tl) {
                //throw exception
                throw std::runtime_error("");
            }

            //transform is good, update the bottom right frame (to frame)
            std::string new_br = t.br;
            return IndexedTransform(
                tl,
                bl,
                new_br,
                new_transform
            );
        }
    }
    else {
        //we are global motion
        //if this is global (left hand side), the RHS (t) can be either relative
        //or global as long as the transforms match up
        //t's origin must match this (LHS) origin
        //t's to frame must match this (LHS) from frame
        if((t.Origin() != Origin()) || (t.To() != From())) {
             throw std::runtime_error("");
        }

        if(t.isCoordinateTransform()) {
            std::string new_bl = t.From();
            //this should match the origin
            assert(new_bl == Origin());
            std::string new_br = t.To();
            return IndexedTransform(
                Origin(),
                new_bl,
                new_br,
                new_transform
            );
        }
        else {
            std::string new_bl = t.From();
            std::string new_br = From();
            return IndexedTransform(
                Origin(),
                new_bl,
                new_br,
                new_transform
            );

        }
    }


  }

  This operator*(const This& t) const {
    return compose(t);
  }

  This decompose(const This& t) const {
    return inverse().compose(t);
  }

  //update this one?
  //maybe an invert and inverse (invert operates on this one)
  This inverse() const {
    This tmp = *this;
    tmp.transform = indexed_transform_traits<T>::Inverse(transform);

    //always swap bottom index's
    std::swap(tmp.bl, tmp.br);

    //coordinate transform, this we also update the top left with the new bottom left
    if(isCoordinateTransform()) {
        tmp.tl = tmp.bl;
    }
    return tmp;
  }

};

TEST(CodeConcepts, threeIndexNotationRelativeInverse) {
    gtsam::Pose3 A_B(gtsam::Rot3::Rodrigues(0.1,-0.2,0.01),gtsam::Point3(0.5,3,1));
    IndexedTransform<gtsam::Pose3> a_b("a", "a", "b", A_B);
    EXPECT_TRUE(a_b.isCoordinateTransform());

    auto b_a = a_b.inverse();
    EXPECT_EQ(b_a.Origin(), "b");
    EXPECT_EQ(b_a.From(), "b");
    EXPECT_EQ(b_a.To(), "a");

    EXPECT_EQ(a_b.Origin(), "a");
    EXPECT_EQ(a_b.From(), "a");
    EXPECT_EQ(a_b.To(), "b");


    EXPECT_TRUE(b_a.isCoordinateTransform());
    EXPECT_TRUE(gtsam::assert_equal(A_B.inverse(), b_a.transform));

}

TEST(CodeConcepts, threeIndexNotationAbsoluteInverse) {
    gtsam::Pose3 W_AB(gtsam::Rot3::Rodrigues(0.1,-0.2,0.01),gtsam::Point3(0.5,3,1));
    IndexedTransform<gtsam::Pose3> w_ab("w", "a", "b", W_AB);
    EXPECT_FALSE(w_ab.isCoordinateTransform());

    auto w_ba = w_ab.inverse();
    EXPECT_EQ(w_ba.Origin(), "w");
    EXPECT_EQ(w_ba.From(), "b");
    EXPECT_EQ(w_ba.To(), "a");

    EXPECT_EQ(w_ab.Origin(), "w");
    EXPECT_EQ(w_ab.From(), "a");
    EXPECT_EQ(w_ab.To(), "b");


    EXPECT_FALSE(w_ba.isCoordinateTransform());
    EXPECT_TRUE(gtsam::assert_equal(W_AB.inverse(), w_ba.transform));

}


TEST(CodeConcepts, threeIndexNotationRelativeComposition) {
    gtsam::Pose3 A(gtsam::Rot3::Rodrigues(0.3,2,3),gtsam::Point3(1,2,2));
    gtsam::Pose3 A_B(gtsam::Rot3::Rodrigues(0.1,-0.2,0.01),gtsam::Point3(0.5,3,1));
    IndexedTransform<gtsam::Pose3> a("w", "w", "a", A);
    IndexedTransform<gtsam::Pose3> a_b("a", "a", "b", A_B);

    auto result = a * a_b;
    EXPECT_EQ(result.Origin(), "w");
    EXPECT_EQ(result.From(), "w");
    EXPECT_EQ(result.To(), "b");
    EXPECT_TRUE(gtsam::assert_equal(A * A_B, result.transform));
}


//previsitor
struct Node {
    void operator()(
        const boost::shared_ptr<gtsam::ISAM2Clique>& clique) {
            auto conditional = clique->conditional();
            //it is FACTOR::const_iterator
            for(auto it = conditional->beginFrontals(); it != conditional->endFrontals(); it++) {
                LOG(INFO) << dyno::DynoLikeKeyFormatter(*it);
            }
        }
};


// TEST(CodeConcepts, getISAM2Ordering) {
//     using namespace gtsam;
//     Cal3_S2::shared_ptr K(new Cal3_S2(50.0, 50.0, 0.0, 50.0, 50.0));

//   // Define the camera observation noise model, 1 pixel stddev
//   auto measurementNoise = noiseModel::Isotropic::Sigma(2, 1.0);

//   // Create the set of ground-truth landmarks
//   std::vector<Point3> points = createPoints();

//   // Create the set of ground-truth poses
//   std::vector<Pose3> poses = createPoses();

//   // Create an iSAM2 object. Unlike iSAM1, which performs periodic batch steps
//   // to maintain proper linearization and efficient variable ordering, iSAM2
//   // performs partial relinearization/reordering at each step. A parameter
//   // structure is available that allows the user to set various properties, such
//   // as the relinearization threshold and type of linear solver. For this
//   // example, we we set the relinearization threshold small so the iSAM2 result
//   // will approach the batch result.
//   ISAM2Params parameters;
//   parameters.relinearizeThreshold = 0.01;
//   parameters.relinearizeSkip = 1;
//   ISAM2 isam(parameters);

//   // Create a Factor Graph and Values to hold the new data
//   NonlinearFactorGraph graph;
//   Values initialEstimate;

//   // Loop over the poses, adding the observations to iSAM incrementally
//   for (size_t i = 0; i < poses.size(); ++i) {
//     // Add factors for each landmark observation
//     for (size_t j = 0; j < points.size(); ++j) {
//       PinholeCamera<Cal3_S2> camera(poses[i], *K);
//       Point2 measurement = camera.project(points[j]);
//       graph.emplace_shared<GenericProjectionFactor<Pose3, Point3, Cal3_S2> >(
//           measurement, measurementNoise, Symbol('x', i), Symbol('l', j), K);
//     }

//     // Add an initial guess for the current pose
//     // Intentionally initialize the variables off from the ground truth
//     static Pose3 kDeltaPose(Rot3::Rodrigues(-0.1, 0.2, 0.25),
//                             Point3(0.05, -0.10, 0.20));
//     initialEstimate.insert(Symbol('x', i), poses[i] * kDeltaPose);

//     // If this is the first iteration, add a prior on the first pose to set the
//     // coordinate frame and a prior on the first landmark to set the scale Also,
//     // as iSAM solves incrementally, we must wait until each is observed at
//     // least twice before adding it to iSAM.
//     if (i == 0) {
//       // Add a prior on pose x0, 30cm std on x,y,z and 0.1 rad on roll,pitch,yaw
//       static auto kPosePrior = noiseModel::Diagonal::Sigmas(
//           (Vector(6) << Vector3::Constant(0.1), Vector3::Constant(0.3))
//               .finished());
//       graph.addPrior(Symbol('x', 0), poses[0], kPosePrior);

//       // Add a prior on landmark l0
//       static auto kPointPrior = noiseModel::Isotropic::Sigma(3, 0.1);
//       graph.addPrior(Symbol('l', 0), points[0], kPointPrior);

//       // Add initial guesses to all observed landmarks
//       // Intentionally initialize the variables off from the ground truth
//       static Point3 kDeltaPoint(-0.25, 0.20, 0.15);
//       for (size_t j = 0; j < points.size(); ++j)
//         initialEstimate.insert<Point3>(Symbol('l', j), points[j] + kDeltaPoint);

//     } else {
//       // Update iSAM with the new factors
//       isam.update(graph, initialEstimate);
//       // Each call to iSAM2 update(*) performs one iteration of the iterative
//       // nonlinear solver. If accuracy is desired at the expense of time,
//       // update(*) can be called additional times to perform multiple optimizer
//       // iterations every step.
//       isam.update();
//       Values currentEstimate = isam.calculateEstimate();

//       // Clear the factor graph and values for the next iteration
//       graph.resize(0);
//       initialEstimate.clear();
//     }
//   }

//     // isam.saveGraph(dyno::getOutputFilePath("test_bayes_tree.dot"));

//     std::function<void(const boost::shared_ptr<gtsam::ISAM2Clique>&)> node_func = Node();
//     // dyno::factor_graph_tools::travsersal::depthFirstTraversalEliminiationOrder(isam, node_func);
//     LOG(INFO) << dyno::container_to_string(dyno::factor_graph_tools::travsersal::getEliminatonOrder(isam));
//     // Data rootdata;
//     // Node preVisitor;
//     // no_op postVisitor;
//     // gtsam::treeTraversal::DepthFirstForest(isam, rootdata, preVisitor, postVisitor);

// }

// //https://github.com/zhixy/SolveAXXB/blob/master/axxb/conventionalaxxbsvdsolver.cc
// TEST(CodeConcepts, sylvesterEquation) {
//     // Eigen::MatrixXd A;
//     // A.resize(4, 4);
//     // A << 1, 0, 2, 3,
//     //      4, 1, 0, 2,
//     //      0, 5, 5, 6,
//     //      1, 7, 9, 0;

//     gtsam::Pose3 L_k_1 = dyno::utils::createRandomAroundIdentity<gtsam::Pose3>(0.1);
//     gtsam::Pose3 L_k_1_H_k = dyno::utils::createRandomAroundIdentity<gtsam::Pose3>(0.03);

//     //L_k = L_K_1 * ^{L_k}_kH_{k-1}
//     gtsam::Pose3 L_k = L_k_1 * L_k_1_H_k ;
//     gtsam::Pose3 W_k_1_H_k = L_k_1 * L_k_1_H_k * L_k_1.inverse();



//     Eigen::MatrixXd A = W_k_1_H_k.matrix();
//     Eigen::MatrixXd B = L_k_1_H_k.matrix();

//     //1 as we only have one A here
//     Eigen::MatrixXd m = Eigen::MatrixXd::Zero(12*1,12);
//     Eigen::VectorXd b = Eigen::VectorXd::Zero(12*1);

//     Eigen::Matrix3d Ra = A.topLeftCorner(3,3);
//     Eigen::Vector3d Ta = A.topRightCorner(3,1);
//     Eigen::Matrix3d Rb = B.topLeftCorner(3,3);
//     Eigen::Vector3d Tb = B.topRightCorner(3,1);

//     m.block<9,9>(12*0,0) = Eigen::MatrixXd::Identity(9,9) - Eigen::kroneckerProduct(Ra,Rb);
//     m.block<3,9>(12*0+9,0) = Eigen::kroneckerProduct(Eigen::MatrixXd::Identity(3,3),Tb.transpose());
//     m.block<3,3>(12*0+9,9) = Eigen::MatrixXd::Identity(3,3) - Ra;
//     b.block<3,1>(12*0+9,0) = Ta;

//     // //different to skew in gtsam...
//     // auto skew =[](const Eigen::Vector3d& u) {
//     //     Eigen::Matrix3d u_hat = Eigen::MatrixXd::Zero(3,3);
//     //     u_hat(0,1) = u(2);
//     //     u_hat(1,0) = -u(2);
//     //     u_hat(0,2) = -u(1);
//     //     u_hat(2,0) = u(1);
//     //     u_hat(1,2) = u(0);
//     //     u_hat(2,1) = -u(0);
//     //     return u_hat;
//     // };
//     // Eigen::Matrix3d Ta_skew = skew(Ta);
//     // // m.block<3,9>(12*0+9,0) = Eigen::kroneckerProduct(Eigen::MatrixXd::Identity(3,3),Tb.transpose());
//     // // m.block<3,3>(12*0+9,9) = Eigen::MatrixXd::Identity(3,3) - Ra;
//     // // b.block<3,1>(12*0+9,0) = Ta;
//     // m.block<3,9>(12*0+9,0) = Eigen::kroneckerProduct(Ta_skew,Tb.transpose());
//     // m.block<3,3>(12*0+9,9) = Ta_skew - Ta_skew*Ra;

//     // Eigen::JacobiSVD<Eigen::MatrixXd> svd( m, Eigen::ComputeFullV | Eigen::ComputeFullU );
//     // CHECK(svd.computeV())<<"fail to compute V";

//     Eigen::Matrix<double, 12, 1> x = m.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);
//     Eigen::Matrix3d R = Eigen::Map< Eigen::Matrix<double, 3, 3, Eigen::RowMajor> >(x.data()); //row major

//     LOG(INFO) << x;

//     Eigen::JacobiSVD<Eigen::MatrixXd> svd(R, Eigen::ComputeThinU | Eigen::ComputeThinV);
//     gtsam::Matrix44 handeyetransformation = gtsam::Matrix44::Identity();
//     handeyetransformation.topLeftCorner(3,3) = svd.matrixU() * svd.matrixV().transpose();
//     handeyetransformation.topRightCorner(3,1) = x.block<3,1>(9,0);

//     handeyetransformation.topRightCorner(3,1) = x.block<3,1>(9,0);


//     // Eigen::Matrix3d R_alpha;
//     // R_alpha.row(0) = svd.matrixV().block<3,1>(0,11).transpose();
//     // R_alpha.row(1) = svd.matrixV().block<3,1>(3,11).transpose();
//     // R_alpha.row(2) = svd.matrixV().block<3,1>(6,11).transpose();
//     // //double a = std::fabs(R_alpha.determinant());
//     // //double alpha = R_alpha.determinant()/(pow(std::fabs(R_alpha.determinant()),4./3.));
//     // double det = R_alpha.determinant();
//     // double alpha = std::pow(std::abs(det),4./3.)/det;
//     // Eigen::HouseholderQR<Eigen::Matrix3d> qr(R_alpha/alpha);

//     // gtsam::Matrix44 handeyetransformation = gtsam::Matrix44::Identity();
//     // Eigen::Matrix3d Q = qr.householderQ();
//     // Eigen::Matrix3d Rwithscale = alpha*Q.transpose()*R_alpha;
//     // Eigen::Vector3d R_diagonal = Rwithscale.diagonal();
//     // for(int i=0;i<3;i++)
//     // {
//     //     handeyetransformation.block<3,1>(0,i) = int(R_diagonal(i)>=0?1:-1)*Q.col(i);
//     // }

//     // handeyetransformation.topRightCorner(3,1) = svd.matrixV().block<3,1>(9,11)/alpha;

//     // LOG(INFO) << A;
//     // // LOG(INFO) << B;
//     // LOG(INFO) << handeyetransformation;


//     // // gtsam::Matrix44 calc_C = A*handeyetransformation - handeyetransformation * B;
//     LOG(INFO) << L_k_1;

//      LOG(INFO) << A*handeyetransformation;
//     LOG(INFO) << handeyetransformation * B;

//     LOG(INFO) << handeyetransformation;



//     // Eigen::RealSchur<Eigen::MatrixXd> SchurA(A);
//     // Eigen::MatrixXd R = SchurA.matrixT();
//     // Eigen::MatrixXd U = SchurA.matrixU();


//     // // Eigen::MatrixXd B = -A.transpose();
//     // Eigen::MatrixXd B;
//     // B.resize(2, 2);
//     // B << 0, -1,
//     //      1, 0;

//     // Eigen::RealSchur<Eigen::MatrixXd> SchurB(B);
//     // Eigen::MatrixXd S = SchurB.matrixT();
//     // Eigen::MatrixXd V = SchurB.matrixU();

//     // //C
//     // // Eigen::MatrixXd I_33 =  gtsam::Matrix33::Zero();
//     // Eigen::MatrixXd C;
//     // C.resize(4, 2);
//     // C << 1, 0,
//     //      2, 0,
//     //      0, 3,
//     //      1, 1;

//     // Eigen::MatrixXd F = (U.adjoint() * C) * V;

//     // Eigen::MatrixXd Y = Eigen::internal::matrix_function_solve_triangular_sylvester(R, S, F);

//     // Eigen::MatrixXd X = (U * Y) * V.adjoint();
//     // LOG(INFO) << "X= " << X;

//     // Eigen::MatrixXd C_calc = A * X + X * B;
//     // LOG(INFO) << "C calc= " << C_calc;

// }


TEST(CodeConcepts, drawInformationMatrix) {
    gtsam::Values initial_estimate;
    gtsam::NonlinearFactorGraph graph;
    const auto model = gtsam::noiseModel::Isotropic::Sigma(3, 1);

    using namespace gtsam::symbol_shorthand;


    // //smaller example
    // gtsam::Pose3 first_pose;
    // graph.emplace_shared<gtsam::NonlinearEquality<gtsam::Pose3> >(X(1), gtsam::Pose3());

    // // create factor noise model with 3 sigmas of value 1
    // // const auto model = gtsam::noiseModel::Isotropic::Sigma(3, 1);
    // // create stereo camera calibration object with .2m between cameras
    // const gtsam::Cal3_S2Stereo::shared_ptr K(
    //     new gtsam::Cal3_S2Stereo(1000, 1000, 0, 320, 240, 0.2));

    // //create and add stereo factors between first pose (key value 1) and the three landmarks
    // // graph.emplace_shared<gtsam::GenericStereoFactor<gtsam::Pose3,gtsam::Point3> >(gtsam::StereoPoint2(520, 480, 440), model, 1, 3, K);
    // // graph.emplace_shared<gtsam::GenericStereoFactor<gtsam::Pose3,gtsam::Point3> >(gtsam::StereoPoint2(120, 80, 440), model, 1, 4, K);
    // // graph.emplace_shared<gtsam::GenericStereoFactor<gtsam::Pose3,gtsam::Point3> >(gtsam::StereoPoint2(320, 280, 140), model, 1, 5, K);

    // // //create and add stereo factors between second pose and the three landmarks
    // // graph.emplace_shared<gtsam::GenericStereoFactor<gtsam::Pose3,gtsam::Point3> >(gtsam::StereoPoint2(570, 520, 490), model, 2, 3, K);
    // // graph.emplace_shared<gtsam::GenericStereoFactor<gtsam::Pose3,gtsam::Point3> >(gtsam::StereoPoint2(70, 20, 490), model, 2, 4, K);
    //  graph.emplace_shared<gtsam::PoseToPointFactor<gtsam::Pose3,gtsam::Point3> >(X(1), L(1), gtsam::Point3(1, 1, 5), model);
    // graph.emplace_shared<gtsam::PoseToPointFactor<gtsam::Pose3,gtsam::Point3> >(X(1), L(2),  gtsam::Point3(1, 1, 5), model);
    // graph.emplace_shared<gtsam::PoseToPointFactor<gtsam::Pose3,gtsam::Point3> >(X(1), L(3), gtsam::Point3::Identity(), model);

    // //create and add stereo factors between second pose and the three landmarks
    // graph.emplace_shared<gtsam::PoseToPointFactor<gtsam::Pose3,gtsam::Point3> >(X(2),L(1), gtsam::Point3::Identity(), model);
    // graph.emplace_shared<gtsam::PoseToPointFactor<gtsam::Pose3,gtsam::Point3> >(X(2), L(2), gtsam::Point3::Identity(), model);
    // // graph.emplace_shared<gtsam::GenericStereoFactor<gtsam::Pose3,gtsam::Point3> >(gtsam::StereoPoint2(320, 270, 115), model, 2, 5, K);

    // // create Values object to contain initial estimates of camera poses and
    // // landmark locations

    // // create and add iniital estimates
    // initial_estimate.insert(X(1), first_pose);
    // initial_estimate.insert(X(2), gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0.1, -0.1, 1.1)));
    // initial_estimate.insert(L(1), gtsam::Point3(1, 1, 5));
    // initial_estimate.insert(L(2), gtsam::Point3(-1, 1, 5));
    // initial_estimate.insert(L(3), gtsam::Point3(1, -0.5, 5));

    // std::string calibration_loc = gtsam::findExampleDataFile("VO_calibration.txt");
    // std::string pose_loc = gtsam::findExampleDataFile("VO_camera_poses_large.txt");
    // std::string factor_loc = gtsam::findExampleDataFile("VO_stereo_factors_large.txt");

    // // read camera calibration info from file
    // // focal lengths fx, fy, skew s, principal point u0, v0, baseline b
    // double fx, fy, s, u0, v0, b;
    // std::ifstream calibration_file(calibration_loc.c_str());
    // std::cout << "Reading calibration info" << std::endl;
    // calibration_file >> fx >> fy >> s >> u0 >> v0 >> b;

    // // create stereo camera calibration object
    // const gtsam::Cal3_S2Stereo::shared_ptr K(new gtsam::Cal3_S2Stereo(fx, fy, s, u0, v0, b));

    // std::ifstream pose_file(pose_loc.c_str());
    // std::cout << "Reading camera poses" << std::endl;
    // int pose_id;
    // gtsam::MatrixRowMajor m(4, 4);
    // // read camera pose parameters and use to make initial estimates of camera
    // // poses
    // while (pose_file >> pose_id) {
    //     for (int i = 0; i < 16; i++) {
    //         pose_file >> m.data()[i];
    //     }
    //     initial_estimate.insert(gtsam::Symbol('x', pose_id), gtsam::Pose3(m));
    // }

    // // camera and landmark keys
    // size_t x, l;

    // // pixel coordinates uL, uR, v (same for left/right images due to
    // // rectification) landmark coordinates X, Y, Z in camera frame, resulting from
    // // triangulation
    // double uL, uR, v, X, Y, Z;
    // std::ifstream factor_file(factor_loc.c_str());
    // std::cout << "Reading stereo factors" << std::endl;
    // // read stereo measurement details from file and use to create and add
    // // GenericStereoFactor objects to the graph representation
    // while (factor_file >> x >> l >> uL >> uR >> v >> X >> Y >> Z) {
    //     graph.emplace_shared<gtsam::GenericStereoFactor<gtsam::Pose3, gtsam::Point3> >(
    //         gtsam::StereoPoint2(uL, uR, v), model, gtsam::Symbol('x', x), gtsam::Symbol('l', l), K);
    //     // if the landmark variable included in this factor has not yet been added
    //     // to the initial variable value estimate, add it
    //     if (!initial_estimate.exists(gtsam::Symbol('l', l))) {
            // gtsam::Pose3 camPose = initial_estimate.at<gtsam::Pose3>(gtsam::Symbol('x', x));
    //         // transformFrom() transforms the input Point3 from the camera pose space,
    //         // camPose, to the global space
    //         gtsam::Point3 worldPoint = camPose.transformFrom(gtsam::Point3(X, Y, Z));
    //         initial_estimate.insert(gtsam::Symbol('l', l), worldPoint);
    //     }
    // }

    // auto gfg = graph.linearize(initial_estimate);
    // // gtsam::Matrix jacobian = gfg->jacobian().first;

    // gtsam::Ordering natural_ordering = gtsam::Ordering::Natural(*gfg);
    // gtsam::JacobianFactor jf(*gfg, natural_ordering);
    // gtsam::Matrix J = jf.jacobian().first;

    // {
    //     gtsam::Values pose_only_values;
    //     pose_only_values.insert(0, gtsam::Pose3::Identity());
    //     pose_only_values.insert(1, gtsam::Pose3::Identity());
    //     gtsam::NonlinearFactorGraph pose_only_graph;
    //     pose_only_graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(0, gtsam::Pose3::Identity(), gtsam::noiseModel::Isotropic::Sigma(6, 1));
    //     pose_only_graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(1, gtsam::Pose3::Identity(), gtsam::noiseModel::Isotropic::Sigma(6, 1));


    //     gtsam::Ordering natural_ordering_pose = gtsam::Ordering::Natural(pose_only_graph);
    //     auto gfg_pose = pose_only_graph.linearize(pose_only_values);
    //     gtsam::JacobianFactor jf_pose(*gfg_pose, natural_ordering_pose);

    //         // auto size = natural_ordering.size();
    //         // auto size1 = jf_pose.cols() * 6;
    //     //cols should be num variables * 6 as each pose has dimenstions variables
    //     EXPECT_EQ(natural_ordering_pose.size() * 6,(jf_pose.getA().cols()));

    //     //this just a view HORIZONTALLY, vertically it is the entire matrix
    //     const gtsam::VerticalBlockMatrix::constBlock Ablock = jf_pose.getA(jf_pose.find(0));
    //     //in this example there should be 2? blocks
    //     // LOG(INFO) << "Num blocks " << Ablock.nBlocks();
    //     LOG(INFO) << Ablock;

    //     for(gtsam::Key key : natural_ordering_pose) {
    //     //this will have rows = J.rows(), cols = dimension of the variable
    //     const gtsam::VerticalBlockMatrix::constBlock Ablock = jf_pose.getA(jf_pose.find(key));
    //     const size_t var_dimensions = Ablock.cols();cv::Vec3b(255, 0, 0);
    //     EXPECT_EQ(Ablock.rows(), jf_pose.rows());

    //     LOG(INFO) << "Start Col " << Ablock.startCol() << " Start row " << Ablock.startRow() << " dims " << var_dimensions;
    //     //eachs start col shoudl be dim(key) apart
    //     // for (int i = 0; i < J.rows(); ++i) {
    //     //     for (int j = 0; j < J.cols(); ++j) {
    //     //         if (std::fabs(J(i, j)) > 1e-15) {
    //     //             // make non zero elements blue
    //     //             J_img.at<cv::Vec3b>(i, j) = cv::Vec3b(255, 0, 0);
    //     //         }
    //     //     }
    //     // }

    //     }

    // }

    // graph.saveGraph(dyno::getOutputFilePath("test_graph.dot"));

    // LOG(INFO) << natural_ordering.size();
    // LOG(INFO) << J.cols();

    // cv::Mat J_img(cv::Size(J.cols(), J.rows()), CV_8UC3, cv::Scalar(255, 255, 255));

    // std::vector<std::pair<gtsam::Key, cv::Mat>> column_blocks;
    // LOG(INFO) << J_img.cols;
    // for(gtsam::Key key : natural_ordering) {
    //     //this will have rows = J.rows(), cols = dimension of the variable
    //     // LOG(INFO) << key;
    //     const gtsam::VerticalBlockMatrix::constBlock Ablock = jf.getA(jf.find(key));
    //     const size_t var_dimensions = Ablock.cols();
    //     EXPECT_EQ(Ablock.rows(), J.rows());

    //     // LOG(INFO) << "Start Col " << Ablock.startCol() << " Start row " << Ablock.startRow() << " dims " << var_dimensions;



    //     for (int i = 0; i < J.rows(); ++i) {
    //         for (int j = Ablock.startCol(); j < (Ablock.startCol() + var_dimensions); ++j) {
    //             ASSERT_LT(j, J_img.cols);
    //             ASSERT_LT(i, J_img.rows);
    //             if (std::fabs(J(i, j)) > 1e-15) {
    //                 // make non zero elements blue
    //                 const auto colour = dyno::ColourMap::getObjectColour((int)var_dimensions);
    //                 J_img.at<cv::Vec3b>(i, j) =  cv::Vec3b(colour[0], colour[1], colour[2]);
    //                 // J_img.at<cv::Vec3b>(i, j) = cv::Vec3b(255, 0, 0);
    //                 // J_img.at<cv::Vec3b>(i, j)[2] = colour[2];
    //             }
    //         }
    //     }

    //     cv::Mat b_img(cv::Size(var_dimensions, Ablock.rows()), CV_8UC3, cv::Scalar(255, 255, 255));
    //     for (int i = 0; i < Ablock.rows(); ++i) {
    //         for (int j = 0; j < var_dimensions; ++j) {
    //             // ASSERT_LT(j, J_img.cols);
    //             // ASSERT_LT(i, J_img.rows);
    //             if (std::fabs(Ablock(i, j)) > 1e-15) {
    //                 // make non zero elements blue
    //                 const auto colour = dyno::ColourMap::getObjectColour((int)var_dimensions);
    //                 b_img.at<cv::Vec3b>(i, j) =  cv::Vec3b(colour[0], colour[1], colour[2]);
    //                 // J_img.at<cv::Vec3b>(i, j) = cv::Vec3b(255, 0, 0);
    //                 // J_img.at<cv::Vec3b>(i, j)[2] = colour[2];
    //             }
    //         }
    //     }

    //     column_blocks.push_back(std::make_pair(key, b_img));

    // }

    // LOG(INFO) << column_blocks.size();

    // cv::Size desired_size(480, 480);

    // const int current_cols = J.cols();
    // const int desired_cols = desired_size.width;
    // const int desired_rows = desired_size.height;

    // double ratio = (double)desired_cols/(double)current_cols;
    // LOG(INFO) << ratio;
    // cv::Mat concat_column_blocks;
    // // cv::Mat concat_column_blocks = column_blocks.at(0);
    // cv::Mat labels;

    // for(size_t i = 1; i < column_blocks.size(); i++) {
    //     gtsam::Key key = column_blocks.at(i).first;
    //     cv::Mat current_block = column_blocks.at(i).second;
    //     int scaled_cols = ratio * (int)current_block.cols;
    //     cv::resize(current_block, current_block, cv::Size(scaled_cols, desired_rows), 0, 0, cv::INTER_NEAREST);

    //     //draw text info
    //     int baseline=0;
    //     // cv::Size textSize = cv::getTextSize(gtsam::DefaultKeyFormatter(key),cv::FONT_HERSHEY_PLAIN,1,1,&baseline);

    //     const int text_box_height = 30;
    //     cv::Mat text_box(cv::Size(current_block.cols, text_box_height), CV_8UC3, cv::Scalar(255, 255, 255));

    //     constexpr static double kFontScale = 0.5;
    //     constexpr static int kFontFace = cv::FONT_HERSHEY_SIMPLEX;
    //     constexpr static int kThickness = 2;
    //     //draw text mid way in box
    //     cv::putText(text_box, gtsam::DefaultKeyFormatter(key), cv::Point(text_box.cols/2, text_box.rows/2), kFontFace, kFontScale, cv::Scalar(0, 0, 0), kThickness);

    //     // cv::imshow("text", text_box);
    //     // cv::waitKey(0);

    //     cv::vconcat(text_box, current_block, current_block);

    //     if(i == 1) {
    //         cv::Mat first_block = column_blocks.at(0).second;
    //         gtsam::Key key_0 = column_blocks.at(0).first;

    //         int first_scaled_cols = ratio * first_block.cols;
    //         cv::resize(first_block, first_block, cv::Size(first_scaled_cols, desired_rows), 0, 0, cv::INTER_NEAREST);

    //         //add text
    //         cv::Mat text_box_first(cv::Size(first_block.cols, text_box_height), CV_8UC3, cv::Scalar(255, 255, 255));
    //         cv::putText(text_box, gtsam::DefaultKeyFormatter(key_0), cv::Point(text_box_first.cols/2, text_box_first.rows/2), kFontFace, kFontScale, cv::Scalar(0, 0, 0), kThickness);

    //         //concat text box on top of jacobian block
    //         cv::vconcat(text_box, first_block, first_block);
    //         //now concat with next jacobian block
    //         cv::hconcat(first_block, current_block, concat_column_blocks);

    //     }
    //     else {
    //         //concat current jacobian block with other blocks
    //         cv::hconcat(concat_column_blocks, current_block, concat_column_blocks);
    //     }

    //     //if not last add vertical line
    //     if(i < column_blocks.size() - 1) {
    //         cv::Mat vert_line(cv::Size(2, concat_column_blocks.rows), CV_8UC3, cv::Scalar(0, 0, 0));
    //         //concat blocks with vertial line
    //         cv::hconcat(concat_column_blocks, vert_line, concat_column_blocks);
    //         // concat_column_blocks = dyno::utils::concatenateImagesHorizontally(concat_column_blocks, vert_line);
    //     }


    //     // //all blocks should have the same number of rows
    //     // double scaled_cols =

    //     // //add vertical line between each variable block
    //     // cv::Mat vert_line(cv::Size(2, concat_column_blocks.rows), CV_8UC3, cv::Scalar(0, 0, 0));
    //     // concat_column_blocks = dyno::utils::concatenateImagesHorizontally(concat_column_blocks, vert_line);
    // }

    // dyno::NonlinearFactorGraphManager nlfg(graph, initial_estimate);

    // cv::Mat J_1 = nlfg.drawBlockJacobian(gtsam::Ordering::NATURAL, dyno::factor_graph_tools::DrawBlockJacobiansOptions::makeDynoSamOptions());

    // for (int i = 0; i < J.rows(); ++i) {
    //     for (int j = 0; j < J.cols(); ++j) {
    //         if (std::fabs(J(i, j)) > 1e-15) {
    //             // make non zero elements blue
    //             J_img.at<cv::Vec3b>(i, j) = cv::Vec3b(255, 0, 0);
    //         }
    //     }
    // }

    // gtsam::Matrix I = jf.information();
    // cv::Mat I_img(cv::Size(I.rows(), I.cols()), CV_8UC3, cv::Scalar(255, 255, 255));
    // for (int i = 0; i < I.rows(); ++i) {
    //     for (int j = 0; j < I.cols(); ++j) {
    //         if (std::fabs(I(i, j)) > 1e-15) {
    //             // make non zero elements blue
    //             I_img.at<cv::Vec3b>(i, j) = cv::Vec3b(255, 0, 0);
    //         }
    //     }
    // }
    // LOG(INFO) << dyno::to_string(concat_column_blocks.size());
    // //resize for viz
    // cv::resize(concat_column_blocks, concat_column_blocks, cv::Size(480, 480), 0, 0, cv::INTER_NEAREST);
    // cv::resize(J_img, J_img, cv::Size(480, 480), 0, 0, cv::INTER_NEAREST);
    // // // cv::resize(I_img, I_img, cv::Size(480, 480), 0, 0, cv::INTER_CUBIC);

    // cv::imshow("Jacobian", concat_column_blocks);
    // cv::imshow("J orig", J_img);
    // cv::imshow("J_1", J_1);
    cv::waitKey(0);

}


TEST(CodeConcepts, testModifyOptionalString) {

    auto modify = [](std::optional<std::reference_wrapper<std::string>> string) {
        if(string) {
           string->get() = "udpated";
        }
    };

    std::string input = "before";
    modify(input);

    EXPECT_EQ(input, "udpated");
}
```

## File: core_utils/test_csv.cc
```
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

#include "dynosam/utils/CsvParser.hpp"

#include <glog/logging.h>
#include <gtest/gtest.h>

using namespace dyno;

TEST(CsvHeader, testZeroConstruction) {
    CsvHeader header;
    EXPECT_EQ(header.size(), 0u);
}

TEST(CsvHeader, testConstruction) {


    CsvHeader header("1", "2", "3");
    EXPECT_EQ(header.size(), 3u);

    EXPECT_EQ(header.at(0), "1");
    EXPECT_EQ(header.at(1), "2");
    EXPECT_EQ(header.at(2), "3");

}

TEST(CsvHeader, testToString) {
    CsvHeader header("1", "2", "3");
    const std::string header_string = "1,2,3";
    EXPECT_EQ(header.toString(","), header_string);

}

TEST(CsvHeader, testToStringOneHeader) {
    CsvHeader header("1");
    const std::string header_string = "1";
    EXPECT_EQ(header.toString(), header_string);

}

TEST(CsvReader, testReadSimpleRowNoHeader) {
    CsvReader::Row row;

    EXPECT_EQ(row.size(), 0);

    std::stringstream ss;
    ss << "jesse, viorela,yiduo,  mik";

    row << ss;
    EXPECT_EQ(row.size(), 4);
    EXPECT_EQ(row[0], "jesse");
    EXPECT_EQ(row[1], "viorela");
    EXPECT_EQ(row[2], "yiduo");
    EXPECT_EQ(row[3], "mik");
}

TEST(CsvReader, testReadSimpleIterator) {

    std::stringstream ss;
    ss << "jesse, viorela,yiduo,  mik\n";
    ss << "nic, jack,will,ryan";

    CsvReader csv_reader(ss);
    auto it = csv_reader.begin();
    CsvReader::Row row = *it;
    EXPECT_EQ(row.size(), 4);
    EXPECT_EQ(row[0], "jesse");
    EXPECT_EQ(row[1], "viorela");
    EXPECT_EQ(row[2], "yiduo");
    EXPECT_EQ(row[3], "mik");

    it++;
    row = *it;
    EXPECT_EQ(row.size(), 4);
    EXPECT_EQ(row[0], "nic");
    EXPECT_EQ(row[1], "jack");
    EXPECT_EQ(row[2], "will");
    EXPECT_EQ(row[3], "ryan");

}


TEST(CsvWriter, testInvalidHeaderConstruction) {
    EXPECT_THROW({CsvWriter(CsvHeader{});}, InvalidCsvHeaderException);
}



TEST(CsvWriter, testBasicWrite) {
    CsvHeader header("frame_id", "timestamp");
    CsvWriter writer(header);

    writer << 0 << 1.1;
    std::stringstream ss;
    writer.write(ss);

    const std::string expected_write = "frame_id,timestamp\n0,1.1";
    EXPECT_EQ(ss.str(), expected_write);
}


TEST(CsvWriter, testWriteMultiLines) {
    CsvHeader header("frame_id", "timestamp");
    CsvWriter writer(header);

    writer << 0 << 1.1 << 1 << 1.3;
    std::stringstream ss;
    writer.write(ss);

    const std::string expected_write = "frame_id,timestamp\n0,1.1\n1,1.3";
    EXPECT_EQ(ss.str(), expected_write);
}
```

## File: core_utils/test_histogram.cc
```
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

#include <glog/logging.h>
#include <gtest/gtest.h>
#include <exception>
#include <limits>

#include "dynosam/utils/Histogram.hpp"

using namespace dyno;

TEST(Histogram, testConstructors) {

    Histogram hist(bh::make_histogram(bh::axis::regular<>(6, -1.0, 2.0)));
    Histogram hist1(bh::make_histogram(bh::axis::regular<>(6, -1.0, 2.0), bh::axis::variable<>({0, 1, 3, 5, 7})));
    Histogram hist2(bh::make_histogram(bh::axis::variable<>({0.0, 1.0, 3.0, 5.0, 7.0, std::numeric_limits<double>::max()})));

    Histogram hist3(bh::make_histogram(bh::axis::variable<>({0.0, 1.0, 3.0, 5.0, 7.0, std::numeric_limits<double>::max()})));

    EXPECT_EQ(hist.histogram_.rank(), 1);
    EXPECT_EQ(hist1.histogram_.rank(), 2);

    auto data = {-0.5, 1.1, 0.3, 1.7, 3.2, 4.0, 4.6, 4.0,7.0, 9.0, 8.1};
    std::for_each(data.begin(), data.end(), std::ref(hist2.histogram_));

    auto data1 = {1.2, 3.3, 4.0};
    std::for_each(data1.begin(), data1.end(), std::ref(hist3.histogram_));

    LOG(INFO) << hist3.toString();

    hist3.histogram_ += hist2.histogram_;
    LOG(INFO) << hist3.toString();
     LOG(INFO) << hist2.toString();


    using json = nlohmann::json;
    json hist_json;
    to_json(hist_json, hist2);
    LOG(INFO) << hist_json;

}
```

## File: core_utils/test_numerical.cc
```
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

#include <glog/logging.h>
#include <gtest/gtest.h>

#include "dynosam/utils/Numerical.hpp"

using namespace dyno;


TEST(Numerical, testChi_squared_quantile) {
    //from scipy.state.chi2
    //chi2.ppf(0.99, 6) = 16.811893829770927

    static constexpr auto r1 =  16.811893829770927;
    EXPECT_DOUBLE_EQ(chi_squared_quantile(6, 0.99), r1);

    // >>> chi2.ppf(0.5, 3)
    // 2.3659738843753377
    static constexpr auto r2 =  2.3659738843753377;
    EXPECT_DOUBLE_EQ(chi_squared_quantile(3, 0.5), r2);


}
```

## File: core_utils/test_structured_containers.cc
```
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

#include "dynosam/common/StructuredContainers.hpp"
#include "dynosam/common/Types.hpp"


#include <glog/logging.h>
#include <gtest/gtest.h>
#include <exception>
#include <vector>
#include <iterator>

using namespace dyno;

using IntFilterIterator = internal::filter_iterator<std::vector<int>>;

/// @brief make definition for testing
template<>
struct std::iterator_traits<IntFilterIterator> : public dyno::internal::filter_iterator_detail<IntFilterIterator::pointer> {};


// //iterator is the pointer type... this is weird naming but is what the c++ standard does
// template<typename _Iterator>
// struct _filter_iterator_detail {
//     //! naming conventions to match those required by iterator traits
//     using value_type = typename std::iterator_traits<_Iterator>::value_type;
//     using reference = typename std::iterator_traits<_Iterator>::reference;
//     using pointer = typename std::iterator_traits<_Iterator>::pointer;
//     using difference_type = typename std::iterator_traits<_Iterator>::difference_type;
//     using iterator_category = std::forward_iterator_tag; //i guess? only forward is defined (++iter) right now
// };

// //in this case iter is the actual iterator (so _Container::iterator or _container::const_iterator)
// template<typename _Iter, typename _Container>
// struct _filter_iterator : public _filter_iterator_detail<typename _Iter::pointer> {

//     using BaseDetail = _filter_iterator_detail<typename _Iter::pointer>;
//     using iterator = _Iter;
//     using typename BaseDetail::value_type;
//     using typename BaseDetail::reference;
//     using typename BaseDetail::pointer;
//     using typename BaseDetail::difference_type;
//     using typename BaseDetail::iterator_category;

//     _filter_iterator(_Container& container,_Iter it) : container_(container), it_(it) {}

//     _Container& container_;
//     iterator it_;

//     reference operator*() { return *it_; }
//     reference operator->() { return *it_; }

//     bool operator==(const _filter_iterator& other) const {
//         return it_ == other.it_;
//     }
//     bool operator!=(const _filter_iterator& other) const { return it_ != other.it_; }

//     bool operator==(const iterator& other) const {
//         return it_ == other;
//     }
//     bool operator!=(const iterator& other) const { return it_ != other; }

//     _filter_iterator& operator++() {
//         // do {
//         //     ++it_;
//         // }
//         // while(is_invalid());
//         ++it_;
//         return *this;
//     }


//     //allows the iterator to be used as a enhanced for loop
//     _filter_iterator begin() { return _filter_iterator(container_, container_.begin()); }
//     _filter_iterator end() { return _filter_iterator(container_, container_.end()); }

//     const _filter_iterator begin() const { return _filter_iterator(container_, container_.begin()); }
//     const _filter_iterator end() const { return _filter_iterator(container_, container_.end()); }


// };

// TEST(FilterIterator, basic) {

//     using T = std::shared_ptr<int>;

//     std::vector<T> v;
//     _filter_iterator<std::vector<T>::iterator, std::vector<T>> fi(v, v.begin());

//     for(std::shared_ptr<int> i : fi) {}


//     const _filter_iterator<std::vector<T>::const_iterator, std::vector<T>> fi_c(v, v.cbegin());
//     for(std::shared_ptr<int> i : fi_c) {}
// }

TEST(FilterIterator, testNormalIteration) {

    std::vector<int> v = {1, 2, 3, 4, 5};
    internal::filter_iterator<std::vector<int>> v_iter(v, [](const int&) -> bool { return true; });

    EXPECT_EQ(*v_iter, 1);
    ++v_iter;
    EXPECT_EQ(*v_iter, 2);
    ++v_iter;
    EXPECT_EQ(*v_iter, 3);
    ++v_iter;
    EXPECT_EQ(*v_iter, 4);
    ++v_iter;
    EXPECT_EQ(*v_iter, 5);
    ++v_iter;
    EXPECT_EQ(v_iter, v.end());
}

TEST(FilterIterator, testConditionalIteratorWithValidStartingIndex) {

    //start with valid element at v(0)
    std::vector<int> v = {2, 3, 4, 5};
    //true on even numbers
    internal::filter_iterator<std::vector<int>> v_iter(v, [](const int& v) -> bool { return v % 2 == 0; });

    EXPECT_EQ(*v_iter, 2);
    ++v_iter;
    EXPECT_EQ(*v_iter, 4);
    ++v_iter;
    EXPECT_EQ(v_iter, v.end());
}

TEST(FilterIterator, testConditionalIteratorAsLoop) {

    //start with valid element at v(0)
    std::vector<int> v = {2, 3, 4, 5};
    //true on even numbers
    internal::filter_iterator<std::vector<int>> v_iter(v, [](const int& v) -> bool { return v % 2 == 0; });

    int index = 0;
    for(const int& i : v_iter) {
        if(index == 0) {
            EXPECT_EQ(i, 2);
        }
        else if(index == 1) {
            EXPECT_EQ(i, 4);
        }
        else {
            FAIL() << "Should not get here";
        }

        index++;

    }

    EXPECT_EQ(index, 2); //2 iterations only!!!
}


TEST(FilterIterator, testConditionalIteratorWithInvalidStartingIndex) {

    //start with valid element at v(0)
    std::vector<int> v = {1, 2, 3, 4, 5, 6};
    //true on even numbers
    internal::filter_iterator<std::vector<int>> v_iter(v, [](const int& v) -> bool { return v % 2 == 0; });

    EXPECT_EQ(*v_iter, 2);
    ++v_iter;
    EXPECT_EQ(*v_iter, 4);
    ++v_iter;
    EXPECT_EQ(*v_iter, 6);
    EXPECT_NE(v_iter, v.end());
    ++v_iter;
    EXPECT_EQ(v_iter, v.end());
}


TEST(FilterIterator, testStdDistance) {

    {
        std::vector<int> v = {1, 2, 3, 4, 5, 6};
        //true on even numbers
        internal::filter_iterator<std::vector<int>> v_iter(v, [](const int& v) -> bool { return v % 2 == 0; });
        EXPECT_EQ(std::distance(v_iter.begin(), v_iter.end()), 3);
    }

    {
        //start with valid element at v(0)
        std::vector<int> v = {1, 2, 3};
        //true on even numbers
        internal::filter_iterator<std::vector<int>> v_iter(v, [](const int& v) -> bool { return v % 2 == 0; });
        EXPECT_EQ(std::distance(v_iter.begin(), v_iter.end()), 1);
    }


     {
        //start with valid element at v(0)
        std::vector<int> v = {2, 2, 2};
        //true on even numbers
        internal::filter_iterator<std::vector<int>> v_iter(v, [](const int& v) -> bool { return v % 2 == 0; });
        EXPECT_EQ(std::distance(v_iter.begin(), v_iter.end()), 3);
    }
}
```

## File: core_utils/test_types.cc
```
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

#include <dynosam/utils/JsonUtils.hpp>
#include <filesystem>
#include <nlohmann/json.hpp>  //for gt packet seralize tests

#include "dynosam/common/Exceptions.hpp"
#include "dynosam/common/GroundTruthPacket.hpp"
#include "dynosam/frontend/FrontendInputPacket.hpp"
#include "dynosam/frontend/vision/Feature.hpp"
#include "dynosam/logger/Logger.hpp"
#include "dynosam/utils/Statistics.hpp"
#include "dynosam/utils/Variant.hpp"
#include "internal/helpers.hpp"

using namespace dyno;

#include <glog/logging.h>
#include <gmock/gmock.h>
#include <gtest/gtest.h>

// //custom type with dyno::to_string defined. Must be inside dyno namespace
// namespace dyno {
//     struct CustomToString {};
// }

// template<>
// std::string dyno::to_string(const CustomToString&) {
//     return "custom_to_string";
// }

// TEST(IOTraits, testToString) {

//     EXPECT_EQ(traits<decltype(4)>::ToString(4), "4");
//     EXPECT_EQ(traits<CustomToString>::ToString(CustomToString{}),
//     "custom_to_string");
// }

// TEST(Exceptions, testExceptionStream) {
//     EXPECT_THROW({ExceptionStream::Create<DynosamException>();},
//     std::runtime_error); EXPECT_NO_THROW({ExceptionStream::Create();});
// }

// TEST(Exceptions, testExceptionStreamMessage) {
//     //would be preferable to use gmock like
//     //Throws<std::runtime_error>(Property(&std::runtime_error::what,
//     //      HasSubstr("message"))));
//     //but currently issues getting the gmock library to be found...
//     // try {
//     //     ExceptionStream::Create<std::runtime_error>() << "A message";
//     // }
//     // catch(const std::runtime_error& expected) {
//     //     EXPECT_EQ(std::string(expected.what()), "A message");
//     // }
//     // catch(...) {
//     //     FAIL() << "An excpetion was thrown but it was not
//     std::runtime_error";
//     // }
//     // FAIL() << "Exception should be thrown but was not";
//     ExceptionStream::Create<std::runtime_error>() << "A message";
// }

// // TEST(Exceptions, testBasicThrow) {
// //     checkAndThrow(false);
// //     // EXPECT_THROW({checkAndThrow(false);}, DynosamException);
// //     // EXPECT_NO_THROW({checkAndThrow(true);});
// // }

TEST(GtsamUtils, isGtsamValueType) {
  EXPECT_TRUE(is_gtsam_value_v<gtsam::Pose3>);
  EXPECT_TRUE(is_gtsam_value_v<gtsam::Point3>);
  // EXPECT_FALSE(is_gtsam_value_v<double>);
  EXPECT_FALSE(is_gtsam_value_v<ImageType::RGBMono>);
}

TEST(VariantTypes, isVariant) {
  using Var = std::variant<int, std::string>;
  EXPECT_TRUE(is_variant_v<Var>);
  EXPECT_FALSE(is_variant_v<int>);
}

TEST(VariantTypes, variantContains) {
  using Var = std::variant<int, std::string>;
  // for some reason EXPECT_TRUE doenst work?
  //  EXPECT_TRUE(isvariantmember_v<int, Var>);
  bool r = is_variant_member_v<int, Var>;
  EXPECT_EQ(r, true);

  r = is_variant_member_v<std::string, Var>;
  EXPECT_EQ(r, true);

  r = is_variant_member_v<double, Var>;
  EXPECT_EQ(r, false);
}

TEST(ImageType, testRGBMonoValidation) {
  {
    // invalid type
    cv::Mat input(cv::Size(50, 50), CV_64F);
    EXPECT_THROW({ ImageType::RGBMono::validate(input); },
                 InvalidImageTypeException);
  }

  {
    // okay type
    cv::Mat input(cv::Size(50, 50), CV_8UC1);
    EXPECT_NO_THROW({ ImageType::RGBMono::validate(input); });
  }

  {
    // okay type
    cv::Mat input(cv::Size(50, 50), CV_8UC3);
    EXPECT_NO_THROW({ ImageType::RGBMono::validate(input); });
  }
}

TEST(ImageType, testDepthValidation) {
  {
    // okay type
    cv::Mat input(cv::Size(50, 50), CV_64F);
    EXPECT_NO_THROW({ ImageType::Depth::validate(input); });
  }

  {
    // invalud type
    cv::Mat input(cv::Size(50, 50), CV_8UC3);
    EXPECT_THROW({ ImageType::Depth::validate(input); },
                 InvalidImageTypeException);
  }
}

TEST(ImageType, testOpticalFlowValidation) {
  {
    // okay type
    cv::Mat input(cv::Size(50, 50), CV_32FC2);
    EXPECT_NO_THROW({ ImageType::OpticalFlow::validate(input); });
  }

  {
    // invalud type
    cv::Mat input(cv::Size(50, 50), CV_8UC3);
    EXPECT_THROW({ ImageType::OpticalFlow::validate(input); },
                 InvalidImageTypeException);
  }
}

TEST(ImageType, testSemanticMaskValidation) {
  // TODO:
}

TEST(ImageType, testMotionMaskValidation) {
  // TODO:
}

TEST(ImageContainerSubset, testBasicSubsetContainer) {
  cv::Mat depth(cv::Size(25, 25), CV_64F);
  uchar* depth_ptr = depth.data;

  cv::Mat optical_flow(cv::Size(25, 25), CV_32FC2);
  uchar* optical_flow_ptr = optical_flow.data;
  ImageContainerSubset<ImageType::Depth, ImageType::OpticalFlow> ics{
      ImageWrapper<ImageType::Depth>(depth),
      ImageWrapper<ImageType::OpticalFlow>(optical_flow)};

  cv::Mat retrieved_depth = ics.get<ImageType::Depth>();
  cv::Mat retrieved_of = ics.get<ImageType::OpticalFlow>();

  EXPECT_EQ(retrieved_depth.data, depth_ptr);
  EXPECT_EQ(retrieved_of.data, optical_flow_ptr);

  EXPECT_EQ(depth.size(), retrieved_depth.size());
  EXPECT_EQ(optical_flow.size(), retrieved_of.size());
}

TEST(ImageContainerSubset, testSubsetContainerExists) {
  cv::Mat depth(cv::Size(25, 25), CV_64F);
  cv::Mat optical_flow(cv::Size(25, 25), CV_32FC2);
  ImageContainerSubset<ImageType::Depth, ImageType::OpticalFlow,
                       ImageType::RGBMono>
      ics{ImageWrapper<ImageType::Depth>(depth),
          ImageWrapper<ImageType::OpticalFlow>(optical_flow),
          ImageWrapper<ImageType::RGBMono>()};

  EXPECT_TRUE(ics.exists<ImageType::Depth>());
  EXPECT_TRUE(ics.exists<ImageType::OpticalFlow>());

  EXPECT_FALSE(ics.exists<ImageType::RGBMono>());
}

TEST(ImageContainerSubset, testBasicMakeSubset) {
  cv::Mat depth(cv::Size(50, 50), CV_64F);
  cv::Mat optical_flow(cv::Size(50, 50), CV_32FC2);
  uchar* optical_flow_ptr = optical_flow.data;
  ImageContainerSubset<ImageType::Depth, ImageType::OpticalFlow> ics{
      ImageWrapper<ImageType::Depth>(depth),
      ImageWrapper<ImageType::OpticalFlow>(optical_flow)};

  EXPECT_EQ(ics.index<ImageType::OpticalFlow>(), 1u);

  ImageContainerSubset<ImageType::OpticalFlow> subset =
      ics.makeSubset<ImageType::OpticalFlow>();
  EXPECT_EQ(subset.index<ImageType::OpticalFlow>(), 0u);

  cv::Mat retrieved_of = ics.get<ImageType::OpticalFlow>();
  cv::Mat subset_retrieved_of = subset.get<ImageType::OpticalFlow>();

  EXPECT_EQ(retrieved_of.data, optical_flow_ptr);
  EXPECT_EQ(subset_retrieved_of.data, optical_flow_ptr);
}

TEST(ImageContainerSubset, testSafeGet) {
  cv::Mat depth(cv::Size(25, 25), CV_64F);
  cv::Mat optical_flow(cv::Size(25, 25), CV_32FC2);
  ImageContainerSubset<ImageType::Depth, ImageType::OpticalFlow,
                       ImageType::RGBMono>
      ics{ImageWrapper<ImageType::Depth>(depth),
          ImageWrapper<ImageType::OpticalFlow>(optical_flow),
          ImageWrapper<ImageType::RGBMono>()};

  cv::Mat tmp;
  EXPECT_FALSE(ics.safeGet<ImageType::RGBMono>(tmp));

  // tmp shoudl now be the same as the depth ptr
  EXPECT_TRUE(ics.safeGet<ImageType::Depth>(tmp));
  EXPECT_EQ(depth.data, tmp.data);
  EXPECT_EQ(depth.size(), tmp.size());

  // tmp shoudl now be the same as the optical flow ptr
  EXPECT_TRUE(ics.safeGet<ImageType::OpticalFlow>(tmp));
  EXPECT_EQ(optical_flow.data, tmp.data);
  EXPECT_EQ(optical_flow.size(), tmp.size());
}

TEST(ImageContainerSubset, testSafeClone) {
  cv::Mat depth(cv::Size(50, 50), CV_64F);
  cv::Mat optical_flow(cv::Size(50, 50), CV_32FC2);
  ImageContainerSubset<ImageType::Depth, ImageType::OpticalFlow,
                       ImageType::RGBMono>
      ics{ImageWrapper<ImageType::Depth>(depth),
          ImageWrapper<ImageType::OpticalFlow>(optical_flow),
          ImageWrapper<ImageType::RGBMono>()};

  cv::Mat tmp;
  EXPECT_FALSE(ics.cloneImage<ImageType::RGBMono>(tmp));
  EXPECT_TRUE(tmp.empty());

  EXPECT_TRUE(ics.cloneImage<ImageType::Depth>(tmp));
  EXPECT_NE(depth.data, tmp.data);  // no longer the same data
  EXPECT_EQ(depth.size(), tmp.size());

  EXPECT_TRUE(ics.cloneImage<ImageType::OpticalFlow>(tmp));
  EXPECT_NE(optical_flow.data, tmp.data);  // no longer the same data
  EXPECT_EQ(optical_flow.size(), tmp.size());
}

TEST(ImageContainer, testImageContainerIndexing) {
  EXPECT_EQ(ImageContainer::Index<ImageType::RGBMono>(), 0u);
  EXPECT_EQ(ImageContainer::Index<ImageType::Depth>(), 1u);
  EXPECT_EQ(ImageContainer::Index<ImageType::OpticalFlow>(), 2u);
  EXPECT_EQ(ImageContainer::Index<ImageType::SemanticMask>(), 3u);
  EXPECT_EQ(ImageContainer::Index<ImageType::MotionMask>(), 4u);
}

TEST(ImageContainer, CreateRGBDSemantic) {
  cv::Mat rgb(cv::Size(50, 50), CV_8UC1);
  cv::Mat depth(cv::Size(50, 50), CV_64F);
  cv::Mat optical_flow(cv::Size(50, 50), CV_32FC2);
  cv::Mat semantic_mask(cv::Size(50, 50), CV_32SC1);
  ImageContainer::Ptr rgbd_semantic = ImageContainer::Create(
      0u, 0u, ImageWrapper<ImageType::RGBMono>(rgb),
      ImageWrapper<ImageType::Depth>(depth),
      ImageWrapper<ImageType::OpticalFlow>(optical_flow),
      ImageWrapper<ImageType::SemanticMask>(semantic_mask));

  EXPECT_TRUE(rgbd_semantic->hasSemanticMask());
  EXPECT_TRUE(rgbd_semantic->hasDepth());
  EXPECT_FALSE(rgbd_semantic->hasMotionMask());
  EXPECT_FALSE(rgbd_semantic->isMonocular());
}

TEST(ImageContainer, CreateRGBDSemanticWithInvalidSizes) {
  cv::Mat rgb(cv::Size(25, 25), CV_8UC1);
  cv::Mat depth(cv::Size(50, 50), CV_64F);
  cv::Mat optical_flow(cv::Size(50, 50), CV_32FC2);
  cv::Mat semantic_mask(cv::Size(50, 50), CV_32SC1);
  EXPECT_THROW(
      {
        ImageContainer::Create(
            0u, 0u, ImageWrapper<ImageType::RGBMono>(rgb),
            ImageWrapper<ImageType::Depth>(depth),
            ImageWrapper<ImageType::OpticalFlow>(optical_flow),
            ImageWrapper<ImageType::SemanticMask>(semantic_mask));
      },
      ImageContainerConstructionException);
}

TEST(FeatureContainer, basicAdd) {
  FeatureContainer fc;
  EXPECT_EQ(fc.size(), 0u);
  EXPECT_FALSE(fc.exists(1));

  Feature f;
  f.trackletId(1);

  fc.add(f);
  EXPECT_EQ(fc.size(), 1u);
  EXPECT_TRUE(fc.exists(1));

  // this implicitly tests map access
  auto fr = fc.getByTrackletId(1);
  EXPECT_TRUE(fr != nullptr);
  EXPECT_EQ(*fr, f);
}

TEST(FeatureContainer, basicRemove) {
  FeatureContainer fc;
  EXPECT_EQ(fc.size(), 0u);

  Feature f;
  f.trackletId(1);

  fc.add(f);
  EXPECT_EQ(fc.size(), 1u);

  fc.remove(1);
  EXPECT_FALSE(fc.exists(1));

  auto fr = fc.getByTrackletId(1);
  EXPECT_TRUE(fr == nullptr);
}

TEST(FeatureContainer, testVectorLikeIteration) {
  FeatureContainer fc;

  for (size_t i = 0; i < 10u; i++) {
    Feature f;
    f.trackletId(i);
    fc.add(f);
  }

  int count = 0;
  for (const auto& i : fc) {
    EXPECT_TRUE(i != nullptr);
    count++;
  }

  EXPECT_EQ(count, 10);
  count = 0;

  fc.remove(0);
  fc.remove(1);

  for (const auto& i : fc) {
    EXPECT_TRUE(i != nullptr);
    EXPECT_TRUE(i->trackletId() != 0 || i->trackletId() != 1);
    count++;
  }

  EXPECT_EQ(count, 8);
}

TEST(FeatureContainer, testusableIterator) {
  FeatureContainer fc;

  for (size_t i = 0; i < 10u; i++) {
    Feature f;
    f.trackletId(i);
    fc.add(f);
    EXPECT_TRUE(f.usable());
  }

  {
    auto usable_iterator = fc.beginUsable();
    EXPECT_EQ(std::distance(usable_iterator.begin(), usable_iterator.end()),
              10);
  }

  fc.markOutliers({3});
  fc.markOutliers({4});

  {
    auto usable_iterator = fc.beginUsable();
    EXPECT_EQ(std::distance(usable_iterator.begin(), usable_iterator.end()), 8);

    for (const auto& i : usable_iterator) {
      EXPECT_TRUE(i->trackletId() != 3 || i->trackletId() != 4);
    }
  }
}

TEST(Feature, checkInvalidState) {
  Feature f;
  EXPECT_TRUE(f.inlier());
  EXPECT_FALSE(f.usable());  // inlier initally but invalid tracking label

  f.trackletId(10);
  EXPECT_TRUE(f.usable());

  f.markInvalid();
  EXPECT_FALSE(f.usable());

  f.trackletId(10u);
  EXPECT_TRUE(f.usable());

  f.markOutlier();
  EXPECT_FALSE(f.usable());
}

TEST(Feature, checkDepth) {
  Feature f;
  EXPECT_FALSE(f.hasDepth());

  f.depth(12.0);
  EXPECT_TRUE(f.hasDepth());
}

TEST(GroundTruthInputPacket, findAssociatedObjectWithIdx) {
  ObjectPoseGT obj01;
  obj01.frame_id_ = 0;
  obj01.object_id_ = 1;

  ObjectPoseGT obj02;
  obj02.frame_id_ = 0;
  obj02.object_id_ = 2;

  ObjectPoseGT obj03;
  obj03.frame_id_ = 0;
  obj03.object_id_ = 3;

  ObjectPoseGT obj11;
  obj11.frame_id_ = 1;
  obj11.object_id_ = 1;

  ObjectPoseGT obj12;
  obj12.frame_id_ = 1;
  obj12.object_id_ = 2;

  GroundTruthInputPacket packet_0;
  packet_0.frame_id_ = 0;
  packet_0.object_poses_.push_back(obj01);
  packet_0.object_poses_.push_back(obj02);
  packet_0.object_poses_.push_back(obj03);

  GroundTruthInputPacket packet_1;
  packet_1.frame_id_ = 1;
  // put in out of order compared to packet_1
  packet_1.object_poses_.push_back(obj12);
  packet_1.object_poses_.push_back(obj11);

  size_t obj_idx, obj_other_idx;
  EXPECT_TRUE(
      packet_0.findAssociatedObject(1, packet_1, obj_idx, obj_other_idx));

  EXPECT_EQ(obj_idx, 0);
  EXPECT_EQ(obj_other_idx, 1);

  EXPECT_TRUE(
      packet_0.findAssociatedObject(2, packet_1, obj_idx, obj_other_idx));

  EXPECT_EQ(obj_idx, 1);
  EXPECT_EQ(obj_other_idx, 0);

  // object 3 is not in packet_1
  EXPECT_FALSE(
      packet_0.findAssociatedObject(3, packet_1, obj_idx, obj_other_idx));
}

TEST(GroundTruthInputPacket, findAssociatedObjectWithPtr) {
  ObjectPoseGT obj01;
  obj01.frame_id_ = 0;
  obj01.object_id_ = 1;

  ObjectPoseGT obj02;
  obj02.frame_id_ = 0;
  obj02.object_id_ = 2;

  ObjectPoseGT obj03;
  obj03.frame_id_ = 0;
  obj03.object_id_ = 3;

  ObjectPoseGT obj11;
  obj11.frame_id_ = 1;
  obj11.object_id_ = 1;

  ObjectPoseGT obj12;
  obj12.frame_id_ = 1;
  obj12.object_id_ = 2;

  GroundTruthInputPacket packet_0;
  packet_0.frame_id_ = 0;
  packet_0.object_poses_.push_back(obj01);
  packet_0.object_poses_.push_back(obj02);
  packet_0.object_poses_.push_back(obj03);

  GroundTruthInputPacket packet_1;
  packet_1.frame_id_ = 1;
  // put in out of order compared to packet_1
  packet_1.object_poses_.push_back(obj12);
  packet_1.object_poses_.push_back(obj11);

  ObjectPoseGT* obj;
  const ObjectPoseGT* obj_other;
  EXPECT_TRUE(packet_0.findAssociatedObject(2, packet_1, &obj, &obj_other));

  EXPECT_TRUE(obj != nullptr);
  EXPECT_TRUE(obj_other != nullptr);

  EXPECT_EQ(obj->object_id_, 2);
  EXPECT_EQ(obj_other->object_id_, 2);

  EXPECT_EQ(obj->frame_id_, 0);
  EXPECT_EQ(obj_other->frame_id_, 1);
}

TEST(SensorTypes, MeasurementWithCovarianceConstructionEmpty) {
  MeasurementWithCovariance<Landmark> measurement;
  EXPECT_FALSE(measurement.hasModel());
}

TEST(SensorTypes, MeasurementWithCovarianceConstructionMeasurement) {
  Landmark lmk(10, 12.4, 0.001);
  MeasurementWithCovariance<Landmark> measurement(lmk);
  EXPECT_FALSE(measurement.hasModel());
  EXPECT_EQ(measurement.measurement(), lmk);
}

TEST(SensorTypes, MeasurementWithCovarianceConstructionMeasurementAndSigmas) {
  Landmark lmk(10, 12.4, 0.001);
  gtsam::Vector3 sigmas;
  sigmas << 0.1, 0.2, 0.3;
  MeasurementWithCovariance<Landmark> measurement(lmk, sigmas);
  EXPECT_TRUE(measurement.hasModel());
  EXPECT_EQ(measurement.measurement(), lmk);

  MeasurementWithCovariance<Landmark>::Covariance cov =
      measurement.covariance();

  MeasurementWithCovariance<Landmark>::Covariance expected_cov =
      sigmas.array().pow(2).matrix().asDiagonal();
  EXPECT_TRUE(gtsam::assert_equal(expected_cov, cov));
}

TEST(SensorTypes, MeasurementWithCovarianceConstructionMeasurementAndCov) {
  Landmark lmk(10, 12.4, 0.001);
  MeasurementWithCovariance<Landmark>::Covariance expected_cov;
  expected_cov << 0.1, 0, 0, 0, 0.2, 0, 0, 0, 0.4;
  MeasurementWithCovariance<Landmark> measurement(lmk, expected_cov);
  EXPECT_TRUE(measurement.hasModel());
  EXPECT_EQ(measurement.measurement(), lmk);

  MeasurementWithCovariance<Landmark>::Covariance cov =
      measurement.covariance();
  EXPECT_TRUE(gtsam::assert_equal(expected_cov, cov));
}

TEST(JsonIO, ReferenceFrameValue) {
  ReferenceFrameValue<gtsam::Pose3> ref_frame(gtsam::Pose3::Identity(),
                                              ReferenceFrame::GLOBAL);

  using json = nlohmann::json;
  json j = ref_frame;

  auto ref_frame_load = j.template get<ReferenceFrameValue<gtsam::Pose3>>();
  // TODO: needs equals operator
  //  EXPECT_EQ(kp_load, kp);
}

TEST(JsonIO, ObjectPoseGTIO) {
  ObjectPoseGT object_pose_gt;

  object_pose_gt.frame_id_ = 0;
  object_pose_gt.object_id_ = 1;
  object_pose_gt.L_camera_ = gtsam::Pose3::Identity();
  object_pose_gt.L_world_ = gtsam::Pose3::Identity();
  object_pose_gt.prev_H_current_L_ = gtsam::Pose3::Identity();

  using json = nlohmann::json;
  json j = object_pose_gt;

  auto object_pose_gt_2 = j.template get<ObjectPoseGT>();
  EXPECT_EQ(object_pose_gt, object_pose_gt_2);
}

TEST(JsonIO, MeasurementWithCovSigmas) {
  using json = nlohmann::json;
  Landmark lmk(10, 12.4, 0.001);
  MeasurementWithCovariance<Landmark>::Covariance expected_cov;
  expected_cov << 0.1, 0, 0, 0, 0.2, 0, 0, 0, 0.4;
  MeasurementWithCovariance<Landmark> measurement(lmk, expected_cov);
  json j = measurement;
  auto measurements_load =
      j.template get<MeasurementWithCovariance<Landmark>>();
  EXPECT_TRUE(gtsam::assert_equal(measurements_load, measurement));
}

TEST(JsonIO, MeasurementWithCov) {
  using json = nlohmann::json;
  Landmark lmk(10, 12.4, 0.001);
  gtsam::Vector3 sigmas;
  sigmas << 0.1, 0.2, 0.3;
  MeasurementWithCovariance<Landmark> measurement(lmk, sigmas);

  json j = measurement;
  auto measurements_load =
      j.template get<MeasurementWithCovariance<Landmark>>();
  EXPECT_TRUE(gtsam::assert_equal(measurements_load, measurement));
}

TEST(JsonIO, MeasurementWithNoCov) {
  using json = nlohmann::json;
  Landmark lmk(10, 12.4, 0.001);
  MeasurementWithCovariance<Landmark> measurement(lmk);

  json j = measurement;
  auto measurements_load =
      j.template get<MeasurementWithCovariance<Landmark>>();
  EXPECT_TRUE(gtsam::assert_equal(measurements_load, measurement));
}

TEST(JsonIO, TrackedValueStatusKp) {
  KeypointStatus kp =
      dyno_testing::makeStatusKeypointMeasurement(4, 3, 1, Keypoint(0, 1));

  using json = nlohmann::json;
  json j = (KeypointStatus)kp;

  auto kp_load = j.template get<KeypointStatus>();
  // TODO: needs equals operator
  EXPECT_EQ(kp_load, kp);
}

TEST(JsonIO, TrackedValueStatusKps) {
  StatusKeypointVector measurements;
  for (size_t i = 0; i < 10; i++) {
    measurements.push_back(
        dyno_testing::makeStatusKeypointMeasurement(i, background_label, 0));
  }
  using json = nlohmann::json;
  json j = measurements;

  auto measurements_load = j.template get<StatusKeypointVector>();
  EXPECT_EQ(measurements, measurements);
}

TEST(JsonIO, RGBDInstanceOutputPacket) {
  auto scenario = dyno_testing::makeDefaultScenario();

  std::map<FrameId, RGBDInstanceOutputPacket> rgbd_output;

  for (size_t i = 0; i < 10; i++) {
    auto output = scenario.getOutput(i);
    rgbd_output.insert({i, *output});
  }

  using json = nlohmann::json;
  json j = rgbd_output;
  std::map<FrameId, RGBDInstanceOutputPacket> rgbd_output_loaded =
      j.template get<std::map<FrameId, RGBDInstanceOutputPacket>>();
  EXPECT_EQ(rgbd_output_loaded, rgbd_output);
}

TEST(JsonIO, GroundTruthInputPacketIO) {
  GroundTruthInputPacket gt_packet;

  using json = nlohmann::json;
  json j = gt_packet;

  auto gt_packet_2 = j.template get<GroundTruthInputPacket>();
}

TEST(JsonIO, GroundTruthPacketMapIO) {
  ObjectPoseGT obj01;
  obj01.frame_id_ = 0;
  obj01.object_id_ = 1;

  ObjectPoseGT obj02;
  obj02.frame_id_ = 0;
  obj02.object_id_ = 2;

  ObjectPoseGT obj03;
  obj03.frame_id_ = 0;
  obj03.object_id_ = 3;

  ObjectPoseGT obj11;
  obj11.frame_id_ = 1;
  obj11.object_id_ = 1;

  ObjectPoseGT obj12;
  obj12.frame_id_ = 1;
  obj12.object_id_ = 2;

  GroundTruthInputPacket packet_0;
  packet_0.frame_id_ = 0;
  packet_0.object_poses_.push_back(obj01);
  packet_0.object_poses_.push_back(obj02);
  packet_0.object_poses_.push_back(obj03);

  GroundTruthInputPacket packet_1;
  packet_1.frame_id_ = 1;
  // put in out of order compared to packet_1
  packet_1.object_poses_.push_back(obj12);
  packet_1.object_poses_.push_back(obj11);

  GroundTruthPacketMap gt_packet_map;
  gt_packet_map.insert2(0, packet_0);
  gt_packet_map.insert2(1, packet_1);

  using json = nlohmann::json;
  json j = gt_packet_map;

  auto gt_packet_map_2 = j.template get<GroundTruthPacketMap>();
  EXPECT_EQ(gt_packet_map, gt_packet_map_2);
}

TEST(JsonIO, eigenJsonIO) {
  Eigen::Matrix4d m;
  m << 1.0, 2.0, 3.0, 4.0, 11.0, 12.0, 13.0, 14.0, 21.0, 22.0, 23.0, 24.0, 31.0,
      32.0, 33.0, 34.0;
  nlohmann::json j = m;
  // std::cerr << j.dump() << std::endl;
  Eigen::Matrix4d m2 = j.get<Eigen::Matrix4d>();

  EXPECT_TRUE(gtsam::assert_equal(m, m2));
}

namespace fs = std::filesystem;
class JsonIOWithFiles : public ::testing::Test {
 public:
  JsonIOWithFiles() {}

 protected:
  virtual void SetUp() { fs::create_directory(sandbox); }
  virtual void TearDown() { fs::remove_all(sandbox); }

  const fs::path sandbox{"/tmp/sandbox_json"};
};

TEST_F(JsonIOWithFiles, testSimpleBison) {
  StatusKeypointVector measurements;
  for (size_t i = 0; i < 10; i++) {
    measurements.push_back(
        dyno_testing::makeStatusKeypointMeasurement(i, background_label, 0));
  }

  fs::path tmp_bison_path = sandbox / "simple_bison.bson";
  std::string tmp_bison_path_str = tmp_bison_path;

  JsonConverter::WriteOutJson(measurements, tmp_bison_path_str,
                              JsonConverter::Format::BSON);

  StatusKeypointVector measurements_read;
  EXPECT_TRUE(JsonConverter::ReadInJson(measurements_read, tmp_bison_path_str,
                                        JsonConverter::Format::BSON));
  EXPECT_EQ(measurements_read, measurements);
}

TEST(TrackedValueStatus, testIsTimeInvariant) {
  TrackedValueStatus<Keypoint> status_time_invariant(
      MeasurementWithCovariance<Keypoint>{Keypoint()},
      TrackedValueStatus<Keypoint>::MeaninglessFrame, 0, 0,
      ReferenceFrame::GLOBAL);

  EXPECT_TRUE(status_time_invariant.isTimeInvariant());

  TrackedValueStatus<Keypoint> status_time_variant(
      MeasurementWithCovariance<Keypoint>{Keypoint()},
      0,  // use zero
      0, 0, ReferenceFrame::GLOBAL);
  EXPECT_FALSE(status_time_variant.isTimeInvariant());
}

TEST(Statistics, testGetModules) {
  utils::StatsCollector("global_stats").IncrementOne();
  utils::StatsCollector("ns.spin").IncrementOne();
  utils::StatsCollector("ns.spin1").IncrementOne();

  EXPECT_EQ(utils::Statistics::getTagByModule(),
            std::vector<std::string>({"global_stats"}));
  EXPECT_EQ(utils::Statistics::getTagByModule("ns"),
            std::vector<std::string>({"ns.spin", "ns.spin1"}));
}
```

## File: data/sensor.yaml
```yaml
# %YAML:1.0
# General sensor definitions.
camera_id: camera

# Sensor extrinsics wrt. the body-frame.
T_BS:
  [0.0148655429818, -0.999880929698, 0.00414029679422, -0.0216401454975,
         0.999557249008, 0.0149672133247, 0.025715529948, -0.064676986768,
        -0.0257744366974, 0.00375618835797, 0.999660727178, 0.00981073058949,
         0.0, 0.0, 0.0, 1.0]

# Camera specific definitions.
rate_hz: 20
resolution:
  width: 752
  height: 480
camera_model: pinhole
intrinsics: [458.654, 457.296, 367.215, 248.375] #fu, fv, cu, cv
distortion_model: radial-tangential
distortion_coefficients: [-0.28340811, 0.07395907, 0.00019359, 1.76187114e-05]
```

## File: data/tracking_params.yaml
```yaml
orb_params:
  scale_factor: 12

use_clahe_filter: false
```

## File: dataproviders/ZEDDataProvider/test_ZED_data_provider_conversion_and_mask.cc
```
#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "dynosam/dataprovider/ZEDDataProvider.hpp"
#include "dynosam/common/ZEDCamera.hpp" // For common::ZEDCamera
#include "dynosam/common/ImageTypes.hpp"
#include "dynosam/utils/OpenCVUtils.hpp" // For cvTypeToString

// Mock for common::ZEDCamera
class MockZEDCamera : public dyno::common::ZEDCamera {
public:
    // Explicitly inherit constructors from the base class
    using dyno::common::ZEDCamera::ZEDCamera;


    MOCK_METHOD(bool, open, (), (override));
    MOCK_METHOD(void, close, (), (override));
    MOCK_METHOD(void, shutdown, (), (override));
    MOCK_CONST_METHOD(bool, isOpened, (), (override));
    MOCK_METHOD(sl::ERROR_CODE, grab, (sl::RuntimeParameters&), (override));
    MOCK_METHOD(sl::ERROR_CODE, retrieveRawImage, (sl::VIEW, sl::Mat&, sl::MEM, const sl::Resolution&), (override));
    MOCK_METHOD(sl::ERROR_CODE, retrieveRawDepth, (sl::Mat&, sl::MEM, const sl::Resolution&), (override));
    MOCK_METHOD(sl::ERROR_CODE, retrieveRawMeasure, (sl::MEASURE, sl::Mat&, sl::MEM, const sl::Resolution&), (override));
    MOCK_METHOD(sl::ERROR_CODE, retrieveRawSensorsData, (sl::SensorsData&, sl::TIME_REFERENCE), (override));
    MOCK_METHOD(bool, enableObjectDetection, (), (override));
    MOCK_METHOD(void, disableObjectDetection, (), (override));
    MOCK_CONST_METHOD(bool, isObjectDetectionEnabled, (), (override));
    MOCK_METHOD(sl::ERROR_CODE, retrieveObjects, (sl::Objects&, const sl::ObjectDetectionRuntimeParameters&), (override));
    MOCK_METHOD(bool, enableBodyTracking, (), (override));
    MOCK_METHOD(void, disableBodyTracking, (), (override));
    MOCK_CONST_METHOD(bool, isBodyTrackingEnabled, (), (override));
    MOCK_METHOD(sl::ERROR_CODE, retrieveBodies, (sl::Bodies&, const sl::BodyTrackingRuntimeParameters&), (override));
    MOCK_CONST_METHOD(sl::CameraInformation, getCameraInformation, (), (override));
    MOCK_CONST_METHOD(sl::CalibrationParameters, getCalibrationParameters, (bool), (override));
    MOCK_CONST_METHOD(bool, isSVOMode, (), (override));
    MOCK_METHOD(int, getSVONumberOfFrames, (), (override));
    MOCK_CONST_METHOD(unsigned int, getSVOFrameRate, (), (override));
    MOCK_METHOD(sl::ERROR_CODE, setSVOPosition, (int), (override));
    MOCK_CONST_METHOD(bool, isImuEnabled, (), (override)); // Added for completeness
};

// Test Fixture for ZEDDataProvider Data Conversion and Mask Generation Tests
class ZEDDataProviderConversionTest : public ::testing::Test {
protected:
    std::shared_ptr<MockZEDCamera> mock_zed_camera_;
    std::unique_ptr<dyno::ZEDDataProvider> data_provider_;
    dyno::ZEDConfig test_config_;
    sl::CameraInformation mock_cam_info_;

    void SetUp() override {
        // Default config values can be set here or overridden in tests
        test_config_.resolution = sl::RESOLUTION::HD720;
        test_config_.fps = 30;
        test_config_.depth_mode = sl::DEPTH_MODE::PERFORMANCE;
        test_config_.coordinate_units = sl::UNIT::METER;
        test_config_.coordinate_system_3d_zed = sl::COORDINATE_SYSTEM::IMAGE; // Common for image processing
        test_config_.enable_imu = false;
        test_config_.output_rgb = true; // Default to outputting RGB

        // Mock CameraInformation needed for camera_params_ setup
        mock_cam_info_.camera_configuration.resolution.width = 1280;
        mock_cam_info_.camera_configuration.resolution.height = 720;
        mock_cam_info_.camera_configuration.fps = 30;
        mock_cam_info_.camera_configuration.calibration_parameters.left_cam.fx = 500.0f;
        mock_cam_info_.camera_configuration.calibration_parameters.left_cam.fy = 500.0f;
        mock_cam_info_.camera_configuration.calibration_parameters.left_cam.cx = 640.0f;
        mock_cam_info_.camera_configuration.calibration_parameters.left_cam.cy = 360.0f;
        // Assuming no distortion for ZED rectified images
        for(int i=0; i<5; ++i) mock_cam_info_.camera_configuration.calibration_parameters.left_cam.disto[i] = 0.f;


        // Create a mock ZEDCamera with a default ZEDCameraConfig based on test_config_
        // This mock_zed_camera_ is what the ZEDDataProvider will internally use.
        // We need to ensure the ZEDDataProvider's constructor can accept a ZEDCamera* or ZEDCamera::Ptr.
        // The current ZEDDataProvider constructor creates its own ZEDCamera.
        // To test utility functions in isolation, we might not need the full ZEDDataProvider instance,
        // or we need to refactor ZEDDataProvider to allow injecting a ZEDCamera (which is better for testing).
        // For now, we'll test static or const methods that can be called without complex state.
        // The methods like createImageContainer and generateMaskFromZEDDetections are const,
        // so they can be called on an instance. We'll make a dummy instance.
        // The constructor of ZEDDataProvider creates its own ZEDCamera. This is problematic for mocking.
        // A better design would be to inject the ZEDCamera.
        // For now, we can test the static `slMat_to_cvMat` directly.
        // For `createImageContainer` and `generateMaskFromZEDDetections`, we'll need an instance of `ZEDDataProvider`.
        // We'll construct a ZEDDataProvider, but its internal zed_camera_wrapper_ won't be our mock unless we modify ZEDDataProvider.
        // Let's assume we can test the functions somewhat statically or with minimal state from the provider for now.

        // For methods that are part of ZEDDataProvider instance, we need an instance.
        // The constructor will try to open the camera. We'll mock that part.
        mock_zed_camera_ = std::make_shared<MockZEDCamera>(dyno::ZEDDataProvider::createZEDCameraConfig(test_config_));

        // We will test the static `slMat_to_cvMat` separately.
        // For instance methods, we'll need to create a `ZEDDataProvider`.
        // Its constructor creates a `common::ZEDCamera`. We can't directly inject our mock there
        // without changing `ZEDDataProvider`.
        // However, `createImageContainer` and `generateMaskFromZEDDetections` are `const`.
        // Their main dependency for *these specific tests* is `config_` and `slMat_to_cvMat`.
        // We will create a ZEDDataProvider instance and set its config.
    }

    // Helper to create a ZEDDataProvider instance with a specific config
    // This is a simplified version because the actual ZEDDataProvider constructor
    // creates its own ZEDCamera and tries to open it.
    // For these specific data conversion tests, we primarily need the config
    // and the static slMat_to_cvMat. The instance methods
    // `createImageContainer` and `generateMaskFromZEDDetections` are const.
    void initializeDataProvider(const dyno::ZEDConfig& config) {
        // This is a bit of a workaround. Ideally, ZEDCamera is injected.
        // We are testing const methods that depend mostly on config and static helpers.
        // The actual zed_camera_wrapper_ within this data_provider_ won't be our mock_zed_camera_
        // if the constructor instantiates its own.
        // However, the methods we're testing don't rely on an *opened* camera for their logic.
        data_provider_ = std::make_unique<dyno::ZEDDataProvider>(config, nullptr);
    }
};

// --- Test Cases for ZEDDataProvider: 3. Data Conversion and Mask Generation ---

TEST_F(ZEDDataProviderConversionTest, ZDP_CONV_001_slMatToCvMat_U8C4) {
    sl::Mat sl_mat(10, 10, sl::MAT_TYPE::U8_C4, sl::MEM::CPU);
    // Fill with some data
    for (int i = 0; i < 10 * 10 * 4; ++i) {
        sl_mat.getPtr<sl::uchar1>()[i] = static_cast<sl::uchar1>(i % 256);
    }

    cv::Mat cv_mat = dyno::ZEDDataProvider::slMat_to_cvMat(sl_mat);

    ASSERT_EQ(cv_mat.rows, 10);
    ASSERT_EQ(cv_mat.cols, 10);
    ASSERT_EQ(cv_mat.type(), CV_8UC4);
    for (int r = 0; r < 10; ++r) {
        for (int c = 0; c < 10; ++c) {
            for (int ch = 0; ch < 4; ++ch) {
                ASSERT_EQ(cv_mat.at<cv::Vec4b>(r, c)[ch], sl_mat.getPtr<sl::uchar1>()[(r * 10 + c) * 4 + ch]);
            }
        }
    }
}

TEST_F(ZEDDataProviderConversionTest, ZDP_CONV_002_createImageContainer_BGRAtoBGR) {
    test_config_.output_rgb = true;
    initializeDataProvider(test_config_);

    sl::Mat left_sl(720, 1280, sl::MAT_TYPE::U8_C4, sl::MEM::CPU);
    left_sl.setTo(sl::uchar4(10, 20, 30, 255)); // B, G, R, A
    sl::Mat depth_sl(720, 1280, sl::MAT_TYPE::F32_C1, sl::MEM::CPU);
    depth_sl.setTo(1.5f);

    dyno::ImageContainer::Ptr container = data_provider_->createImageContainer(
        12345.678, 0, left_sl, depth_sl, std::nullopt, std::nullopt);

    ASSERT_TRUE(container);
    const auto& rgb_mono_wrapper = container->get<dyno::ImageType::RGBMono>();
    ASSERT_FALSE(rgb_mono_wrapper.image.empty());
    ASSERT_EQ(rgb_mono_wrapper.image.type(), CV_8UC3) << "Expected BGR (CV_8UC3)";
    cv::Vec3b pixel = rgb_mono_wrapper.image.at<cv::Vec3b>(0,0);
    EXPECT_EQ(pixel[0], 10); // B
    EXPECT_EQ(pixel[1], 20); // G
    EXPECT_EQ(pixel[2], 30); // R
}

TEST_F(ZEDDataProviderConversionTest, ZDP_CONV_003_createImageContainer_BGRAtoGrayscale) {
    test_config_.output_rgb = false;
    initializeDataProvider(test_config_);

    sl::Mat left_sl(720, 1280, sl::MAT_TYPE::U8_C4, sl::MEM::CPU);
    left_sl.setTo(sl::uchar4(10, 20, 30, 255)); // B, G, R, A for a BGR-like input
    sl::Mat depth_sl(720, 1280, sl::MAT_TYPE::F32_C1, sl::MEM::CPU);
    depth_sl.setTo(1.5f);

    dyno::ImageContainer::Ptr container = data_provider_->createImageContainer(
        12345.678, 0, left_sl, depth_sl, std::nullopt, std::nullopt);

    ASSERT_TRUE(container);
    const auto& rgb_mono_wrapper = container->get<dyno::ImageType::RGBMono>();
    ASSERT_FALSE(rgb_mono_wrapper.image.empty());
    ASSERT_EQ(rgb_mono_wrapper.image.type(), CV_8UC1) << "Expected Grayscale (CV_8UC1)";
    // Approximate grayscale value for B=10, G=20, R=30 (standard OpenCV conversion: 0.299R + 0.587G + 0.114B)
    // 0.299*30 + 0.587*20 + 0.114*10 = 8.97 + 11.74 + 1.14 = 21.85
    unsigned char expected_gray = static_cast<unsigned char>(0.114 * 10 + 0.587 * 20 + 0.299 * 30);
    EXPECT_NEAR(rgb_mono_wrapper.image.at<unsigned char>(0,0), expected_gray, 1);
}

TEST_F(ZEDDataProviderConversionTest, ZDP_CONV_004_createImageContainer_DepthF32C1toCV64F) {
    initializeDataProvider(test_config_);

    sl::Mat left_sl(720, 1280, sl::MAT_TYPE::U8_C1, sl::MEM::CPU); // Dummy RGB
    left_sl.setTo(sl::uchar1(128));
    sl::Mat depth_sl(720, 1280, sl::MAT_TYPE::F32_C1, sl::MEM::CPU);
    depth_sl.setTo(2.75f);

    dyno::ImageContainer::Ptr container = data_provider_->createImageContainer(
        12345.678, 0, left_sl, depth_sl, std::nullopt, std::nullopt);

    ASSERT_TRUE(container);
    const auto& depth_wrapper = container->get<dyno::ImageType::Depth>();
    ASSERT_FALSE(depth_wrapper.image.empty());
    ASSERT_EQ(depth_wrapper.image.type(), CV_64FC1) << "Expected Depth (CV_64FC1)";
    EXPECT_DOUBLE_EQ(depth_wrapper.image.at<double>(0,0), 2.75);
}


TEST_F(ZEDDataProviderConversionTest, ZDP_MASK_001_GenerateMask_OD_Segmentation) {
    test_config_.enable_object_detection = true;
    test_config_.object_detection_enable_segmentation = true;
    initializeDataProvider(test_config_);

    sl::Objects zed_objects;
    zed_objects.object_list.resize(1);
    zed_objects.object_list[0].id = 1;
    zed_objects.object_list[0].mask.alloc(10, 10, sl::MAT_TYPE::U8_C1, sl::MEM::CPU);
    zed_objects.object_list[0].mask.setTo(sl::uchar1(0));
    for(int r=2; r<5; ++r) for(int c=2; c<5; ++c) zed_objects.object_list[0].mask.setValue(c,r,sl::uchar1(255));


    dyno::ImageType::MotionMask dummy_mask_type;
    cv::Mat generated_mask = data_provider_->generateMaskFromZEDDetections(
        zed_objects, std::nullopt, cv::Size(10, 10), dummy_mask_type);

    ASSERT_FALSE(generated_mask.empty());
    ASSERT_EQ(generated_mask.type(), CV_32SC1);
    for(int r=0; r<10; ++r) {
        for(int c=0; c<10; ++c) {
            if (r >= 2 && r < 5 && c >= 2 && c < 5) {
                EXPECT_EQ(generated_mask.at<int32_t>(r,c), 1);
            } else {
                EXPECT_EQ(generated_mask.at<int32_t>(r,c), 0);
            }
        }
    }
}

TEST_F(ZEDDataProviderConversionTest, ZDP_MASK_002_GenerateMask_OD_BoundingBox) {
    test_config_.enable_object_detection = true;
    test_config_.object_detection_enable_segmentation = false;
    initializeDataProvider(test_config_);

    sl::Objects zed_objects;
    zed_objects.object_list.resize(1);
    zed_objects.object_list[0].id = 2;
    zed_objects.object_list[0].bounding_box_2d[0] = sl::uint2(1,1); // Top-left
    zed_objects.object_list[0].bounding_box_2d[1] = sl::uint2(5,1); // Top-right
    zed_objects.object_list[0].bounding_box_2d[2] = sl::uint2(5,5); // Bottom-right
    zed_objects.object_list[0].bounding_box_2d[3] = sl::uint2(1,5); // Bottom-left

    dyno::ImageType::MotionMask dummy_mask_type;
    cv::Mat generated_mask = data_provider_->generateMaskFromZEDDetections(
        zed_objects, std::nullopt, cv::Size(10, 10), dummy_mask_type);

    ASSERT_FALSE(generated_mask.empty());
    ASSERT_EQ(generated_mask.type(), CV_32SC1);
     for(int r=0; r<10; ++r) {
        for(int c=0; c<10; ++c) {
            if (r >= 1 && r <= 5 && c >= 1 && c <= 5) { // BBox covers from (1,1) to (5,5)
                EXPECT_EQ(generated_mask.at<int32_t>(r,c), 2);
            } else {
                EXPECT_EQ(generated_mask.at<int32_t>(r,c), 0);
            }
        }
    }
}

TEST_F(ZEDDataProviderConversionTest, ZDP_MASK_003_GenerateMask_BT_Segmentation_WithODOffset) {
    test_config_.enable_body_tracking = true;
    test_config_.body_tracking_enable_segmentation = true;
    test_config_.enable_object_detection = true; // To test ID offset
    initializeDataProvider(test_config_);

    sl::Bodies zed_bodies;
    zed_bodies.body_list.resize(1);
    zed_bodies.body_list[0].id = 1; // Original body ID
    zed_bodies.body_list[0].mask.alloc(10, 10, sl::MAT_TYPE::U8_C1, sl::MEM::CPU);
    zed_bodies.body_list[0].mask.setTo(sl::uchar1(0));
    for(int r=3; r<6; ++r) for(int c=3; c<6; ++c) zed_bodies.body_list[0].mask.setValue(c,r,sl::uchar1(255));

    dyno::ImageType::MotionMask dummy_mask_type;
    cv::Mat generated_mask = data_provider_->generateMaskFromZEDDetections(
        std::nullopt, zed_bodies, cv::Size(10, 10), dummy_mask_type);

    ASSERT_FALSE(generated_mask.empty());
    ASSERT_EQ(generated_mask.type(), CV_32SC1);
    int expected_id = 1 + 1000; // ID offset
    for(int r=0; r<10; ++r) {
        for(int c=0; c<10; ++c) {
            if (r >= 3 && r < 6 && c >= 3 && c < 6) {
                EXPECT_EQ(generated_mask.at<int32_t>(r,c), expected_id);
            } else {
                EXPECT_EQ(generated_mask.at<int32_t>(r,c), 0);
            }
        }
    }
}


TEST_F(ZEDDataProviderConversionTest, ZDP_MASK_004_GenerateMask_OD_and_BT_Combined) {
    test_config_.enable_object_detection = true;
    test_config_.object_detection_enable_segmentation = false; // Use BBox for OD
    test_config_.enable_body_tracking = true;
    test_config_.body_tracking_enable_segmentation = true; // Use Seg for BT
    initializeDataProvider(test_config_);

    sl::Objects zed_objects;
    zed_objects.object_list.resize(1);
    zed_objects.object_list[0].id = 5;
    zed_objects.object_list[0].bounding_box_2d[0] = sl::uint2(0,0);
    zed_objects.object_list[0].bounding_box_2d[1] = sl::uint2(3,0);
    zed_objects.object_list[0].bounding_box_2d[2] = sl::uint2(3,3);
    zed_objects.object_list[0].bounding_box_2d[3] = sl::uint2(0,3);

    sl::Bodies zed_bodies;
    zed_bodies.body_list.resize(1);
    zed_bodies.body_list[0].id = 1; // Original body ID
    zed_bodies.body_list[0].mask.alloc(10, 10, sl::MAT_TYPE::U8_C1, sl::MEM::CPU);
    zed_bodies.body_list[0].mask.setTo(sl::uchar1(0));
    // Body mask overlaps with object BBox but should take precedence due to ID logic
    for(int r=2; r<5; ++r) for(int c=2; c<5; ++c) zed_bodies.body_list[0].mask.setValue(c,r,sl::uchar1(255));


    dyno::ImageType::MotionMask dummy_mask_type;
    cv::Mat generated_mask = data_provider_->generateMaskFromZEDDetections(
        zed_objects, zed_bodies, cv::Size(10, 10), dummy_mask_type);

    ASSERT_FALSE(generated_mask.empty());
    ASSERT_EQ(generated_mask.type(), CV_32SC1);

    int body_id_offset = 1 + 1000;
    for(int r=0; r<10; ++r) {
        for(int c=0; c<10; ++c) {
            if (r >= 2 && r < 5 && c >= 2 && c < 5) { // Body segmentation region
                EXPECT_EQ(generated_mask.at<int32_t>(r,c), body_id_offset) << "Failed at r=" << r << ", c=" << c;
            } else if (r >= 0 && r <= 3 && c >= 0 && c <= 3) { // Object BBox region (not overlapped by body)
                EXPECT_EQ(generated_mask.at<int32_t>(r,c), 5) << "Failed at r=" << r << ", c=" << c;
            } else {
                EXPECT_EQ(generated_mask.at<int32_t>(r,c), 0) << "Failed at r=" << r << ", c=" << c;
            }
        }
    }
}

TEST_F(ZEDDataProviderConversionTest, ZDP_MASK_005_GenerateMask_NoDetections) {
    test_config_.enable_object_detection = true; // Configured, but no data will be provided
    initializeDataProvider(test_config_);

    dyno::ImageType::MotionMask dummy_mask_type;
    cv::Mat generated_mask = data_provider_->generateMaskFromZEDDetections(
        std::nullopt, std::nullopt, cv::Size(10, 10), dummy_mask_type);

    ASSERT_FALSE(generated_mask.empty());
    ASSERT_EQ(generated_mask.type(), CV_32SC1);
    cv::Mat expected_zeros = cv::Mat::zeros(10, 10, CV_32SC1);
    // Check if all elements are zero
    ASSERT_EQ(cv::countNonZero(generated_mask != expected_zeros), 0);
}
```

## File: dataproviders/test_dataset_provider.cc
```
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


#include "dynosam/dataprovider/DatasetProvider.hpp"
#include "dynosam/dataprovider/KittiDataProvider.hpp"
#include "internal/tmp_file.hpp"

#include <glog/logging.h>
#include <gtest/gtest.h>
#include <exception>

double randomGetItem(size_t idx) {
    return idx + 3.5;
}

class DoubleDataFolder : public dyno::DataFolder<double> {

public:
    DoubleDataFolder() {}

    std::string getFolderName() const override {
        return "dummy";
    }


    double getItem(size_t idx) {
        //idk some random equation to show that the value changes?
        return randomGetItem(idx);
    }
};


namespace dyno {

class DatasetProviderFixture : public ::testing::Test
{
public:
  DatasetProviderFixture()
  {
  }

protected:
  virtual void SetUp()
  {
    fs::create_directory(sandbox);
  }
  virtual void TearDown()
  {
    fs::remove_all(sandbox);
  }

  const fs::path sandbox{"/tmp/sandbox"};

};


TEST_F(DatasetProviderFixture, testDataFolderStructureConstructor) {

    DoubleDataFolder::Ptr ddf = std::make_shared<DoubleDataFolder>();
    DataFolderStructure<double> ds("some_path", ddf);

    auto retrieved_ddf = ds.getDataFolder<0>();
    EXPECT_EQ(ddf, retrieved_ddf);
}

TEST_F(DatasetProviderFixture, testDataFolderStructureGetPath) {

    DoubleDataFolder::Ptr ddf = std::make_shared<DoubleDataFolder>();
    DataFolderStructure<double> ds("/tmp/some_path", ddf);

    const std::string folder_path = ds.getAbsoluteFolderPath<0>();
    const std::string expected_folder_path = "/tmp/some_path/dummy";
    EXPECT_EQ(folder_path, expected_folder_path);
}

TEST_F(DatasetProviderFixture, testGenericDatasetLoadingWithValidationFailure) {

    DoubleDataFolder::Ptr ddf = std::make_shared<DoubleDataFolder>();


    EXPECT_THROW({GenericDataset<double>(sandbox, ddf);}, std::runtime_error);
}

TEST_F(DatasetProviderFixture, testGenericDatasetLoadingWithValidationSuccess) {

    DoubleDataFolder::Ptr ddf = std::make_shared<DoubleDataFolder>();

    const fs::path sandbox("/tmp/sandbox");

    std::ofstream{sandbox/ddf->getFolderName()}; // create regular file

    EXPECT_NO_THROW({GenericDataset<double>(sandbox, ddf);});
}


TEST_F(DatasetProviderFixture, testGenericDatasetMockLoading) {

    DoubleDataFolder::Ptr ddf = std::make_shared<DoubleDataFolder>();

    std::ofstream{sandbox/ddf->getFolderName()}; // create regular file

    GenericDataset<double> gd(sandbox, ddf);

    std::vector<double>& data = gd.getDataVector<0>();
    EXPECT_EQ(data.size(), 0u);

    gd.load(1);
    EXPECT_EQ(data.size(), 1u);
    EXPECT_EQ(data.at(0), randomGetItem(0u));

    gd.load(3);
    EXPECT_EQ(data.size(), 3u);
    EXPECT_EQ(data.at(2), randomGetItem(2u));

}

// TEST_F(DatasetProviderFixture, testOptionalGenericdataset) {
//     FunctionalDataFolder<double>::Ptr loader1 = std::make_shared<FunctionalDataFolder<double>>(
//         [=](size_t i) -> double {
//             return (double)i;
//         }
//     );

//     FunctionalDataFolder<int>::Ptr loader2 = std::make_shared<FunctionalDataFolder<int>>(
//         [=](size_t i) ->int {
//             return i;
//         }
//     );

//     GenericDataset<double, std::optional<int>> dataset_with_optional(sandbox, loader1, loader2)
// }


TEST(DynoDataset, testDummy) {

    // KittiDataLoader dd("/root/data/kitti/0000");
    // dd.spin();

}


// TEST(PlaybackGui, creation) {
//     PlaybackGui pg;

//     while(true) pg.draw();
// }



} //dyno
```

## File: internal/helpers.hpp
```
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

#include <gtest/gtest.h>

#include <ament_index_cpp/get_package_prefix.hpp>

#include "dynosam/common/Camera.hpp"
#include "dynosam/common/Types.hpp"
#include "simulator.hpp"

/**
 * @brief gets the full path to the installation directory of the test data
 * which is expected to be at dynosam/test/data
 *
 * The full path will be the ROS install directory of this data after building
 *
 * @return std::string
 */
inline std::string getTestDataPath() {
  return ament_index_cpp::get_package_prefix("dynosam") + "/test/data";
}

namespace dyno_testing {

using namespace dyno;

inline KeypointStatus makeStatusKeypointMeasurement(
    TrackletId tracklet_id, ObjectId object_id, FrameId frame_id,
    const Keypoint& keypoint = Keypoint(), double sigma = 2.0) {
  gtsam::Vector2 kp_sigmas;
  kp_sigmas << sigma, sigma;
  MeasurementWithCovariance<Keypoint> kp_measurement(keypoint, kp_sigmas);
  return KeypointStatus(kp_measurement, frame_id, tracklet_id, object_id,
                        ReferenceFrame::LOCAL);
}

inline void compareLandmarks(const Landmarks& lmks_1, const Landmarks& lmks_2,
                             const float& tol = 1e-9) {
  ASSERT_EQ(lmks_1.size(), lmks_2.size());
  for (size_t i = 0u; i < lmks_1.size(); i++) {
    const auto& lmk_1 = lmks_1[i];
    const auto& lmk_2 = lmks_2[i];
    EXPECT_TRUE(gtsam::assert_equal(lmk_1, lmk_2, tol));
  }
}

inline void compareKeypoints(const Keypoints& lmks_1, const Keypoints& lmks_2,
                             const float& tol = 1e-9) {
  ASSERT_EQ(lmks_1.size(), lmks_2.size());
  for (size_t i = 0u; i < lmks_1.size(); i++) {
    const auto& lmk_1 = lmks_1[i];
    const auto& lmk_2 = lmks_2[i];
    EXPECT_TRUE(gtsam::assert_equal(lmk_1, lmk_2, tol));
  }
}

inline CameraParams makeDefaultCameraParams() {
  CameraParams::IntrinsicsCoeffs intrinsics(4);
  CameraParams::DistortionCoeffs distortion(4);
  intrinsics.at(0) = 721.5377;  // fx
  intrinsics.at(1) = 721.5377;  // fy
  intrinsics.at(2) = 609.5593;  // u0
  intrinsics.at(3) = 172.8540;  // v0
  return CameraParams(intrinsics, distortion, cv::Size(640, 480), "radtan");
}

inline Camera makeDefaultCamera() { return Camera(makeDefaultCameraParams()); }

inline Camera::Ptr makeDefaultCameraPtr() {
  return std::make_shared<Camera>(makeDefaultCameraParams());
}

inline dyno_testing::RGBDScenario makeDefaultScenario() {
  dyno_testing::ScenarioBody::Ptr camera =
      std::make_shared<dyno_testing::ScenarioBody>(
          std::make_unique<dyno_testing::ConstantMotionBodyVisitor>(
              gtsam::Pose3::Identity(),
              // motion only in x
              gtsam::Pose3(gtsam::Rot3::Identity(), gtsam::Point3(0.1, 0, 0))));
  // needs to be at least 3 overlap so we can meet requirements in graph
  // TODO: how can we do 1 point but with lots of overlap (even infinity
  // overlap?)
  dyno_testing::RGBDScenario scenario(
      camera,
      std::make_shared<dyno_testing::SimpleStaticPointsGenerator>(8, 3));

  // add one obect
  const size_t num_points = 3;
  dyno_testing::ObjectBody::Ptr object1 =
      std::make_shared<dyno_testing::ObjectBody>(
          std::make_unique<dyno_testing::ConstantMotionBodyVisitor>(
              gtsam::Pose3(gtsam::Rot3::Identity(), gtsam::Point3(10, 0, 0)),
              // motion only in x
              gtsam::Pose3(gtsam::Rot3::Identity(), gtsam::Point3(0.2, 0, 0))),
          std::make_unique<dyno_testing::ConstantObjectPointsVisitor>(
              num_points));

  dyno_testing::ObjectBody::Ptr object2 =
      std::make_shared<dyno_testing::ObjectBody>(
          std::make_unique<dyno_testing::ConstantMotionBodyVisitor>(
              gtsam::Pose3(gtsam::Rot3::Identity(), gtsam::Point3(10, 0, 0)),
              // motion only in x
              gtsam::Pose3(gtsam::Rot3::Identity(), gtsam::Point3(0.2, 0, 0))),
          std::make_unique<dyno_testing::ConstantObjectPointsVisitor>(
              num_points));

  scenario.addObjectBody(1, object1);
  scenario.addObjectBody(2, object2);
  return scenario;
}

}  // namespace dyno_testing
```

## File: internal/simulator.cc
```
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

#include "simulator.hpp"


namespace dyno_testing {

} //dyno_testing
```

## File: internal/simulator.hpp
```
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
```

## File: internal/tmp_file.cc
```
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

#include "tmp_file.hpp"

#define BOOST_NO_CXX11_SCOPED_ENUMS
#include <boost/filesystem.hpp>
#undef BOOST_NO_CXX11_SCOPED_ENUMS

#include <fstream>
#include <iostream>
#include <mutex>
#include <cstdio>
#include <cstdlib>
#include <glog/logging.h>
#include <unistd.h>

namespace dyno_testing
{

TempFile::TempFile()
{
  relative_path = boost::filesystem::unique_path("%%%%_%%%%_%%%%_%%%%");
  // absolute_path = boost::filesystem::temp_directory_path() / relative_path; //opening temp file does not actually
  // create a file..?
  absolute_path = boost::filesystem::current_path() / relative_path;

  write_stream.open(getFilePath());

  write_stream.close();
}

TempFile::~TempFile()
{
  if (write_stream.is_open())
  {
    write_stream.close();
  }

  if (read_stream.is_open())
  {
    read_stream.close();
  }

  boost::filesystem::remove(absolute_path);
}

std::string TempFile::readLine() const
{
  const std::lock_guard<std::mutex> lg(io_mutex);
  read_stream.open(getFilePath());

  // update the read ptr to the last position
  // TODO:(jesse) ensure the buf is no greater than the length of the file
  std::filebuf* pbuf = read_stream.rdbuf();
  pbuf->pubseekpos(current_pos);

  std::string line;
  std::getline(read_stream, line);

  // set current position to the pos in the file for next read
  current_pos = pbuf->pubseekoff(0, read_stream.cur);

  // if(static_cast<uint8_t>(read_stream.peek()) != 0) {
  //     line += "\n";
  // }

  read_stream.close();
  return line;
}
void TempFile::write(const std::string& data)
{
  const std::lock_guard<std::mutex> lg(io_mutex);

  write_stream.open(getFilePath(), std::ios::app);

  write_stream << data << std::endl;
  write_stream.close();
}

}  // namespace dyno_testing
```

## File: internal/tmp_file.hpp
```
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
#include <iostream>
#include <cstdio>
#include <cstdlib>
#define BOOST_NO_CXX11_SCOPED_ENUMS
#include <boost/filesystem.hpp>
#undef BOOST_NO_CXX11_SCOPED_ENUMS
#include <string>
#include <mutex>
#include <cstdio>
#include <cstdlib>

namespace dyno_testing {
static constexpr std::string_view endl = "\n";


class TempFile
{
public:
  TempFile();
  ~TempFile();

  /**
   * @brief Get the absolute file path
   *
   * @return std::string
   */
  inline std::string getFilePath() const
  {
    return absolute_path.native();
  }

  std::string readLine() const;
  void write(const std::string& data);

private:
  boost::filesystem::path relative_path;
  boost::filesystem::path absolute_path;
  mutable std::ofstream write_stream;
  mutable std::ifstream read_stream;

  std::FILE* tmpf;
  mutable std::mutex io_mutex;
  mutable long current_pos = 0;
};

inline std::ostream& operator<<(std::ostream& os, const TempFile& tmp_file)
{
  os << tmp_file.readLine();
  return os;
}

inline std::istream& operator>>(std::istream& is, TempFile& tmp_file)
{
  std::string input_line;
  std::getline(is, input_line);
  tmp_file.write(input_line);
  return is;
}

inline TempFile& operator<<(TempFile& tmp_file, const std::string& data)
{
  tmp_file.write(data);
  return tmp_file;
}

} // dyno_testing
```

## File: pipelines_params/test_params.cc
```
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

#include "internal/helpers.hpp"
#include "dynosam/frontend/vision/TrackerParams.hpp"


#include <glog/logging.h>
#include <gtest/gtest.h>
#include <config_utilities/parsing/yaml.h>           // Enable fromYamlFile().



using namespace dyno;

TEST(TrackerParams, testLoadingDefault) {

    const std::string file = getTestDataPath() + "/tracking_params.yaml";
    LOG(INFO) << file;

    TrackerParams config = config::fromYamlFile<TrackerParams>(file);

    const std::string config_as_string = config::toString(config);
    std::cout << config_as_string << std::endl;

}
```

## File: pipelines_params/test_pipelines.cc
```
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
#include "internal/simulator.hpp"
#include "internal/helpers.hpp"

#include "dynosam/utils/SafeCast.hpp"

#include "dynosam/pipeline/PipelineBase.hpp"
#include "dynosam/pipeline/PipelinePayload.hpp"
#include "dynosam/common/ModuleBase.hpp"

#include "dynosam/frontend/FrontendPipeline.hpp"
#include "dynosam/backend/RGBDBackendModule.hpp"

#include <filesystem>

#include <glog/logging.h>
#include <gtest/gtest.h>
#include <variant>
#include <type_traits>

using namespace dyno;


namespace fs = std::filesystem;
class FrontendWithFiles : public ::testing::Test
{
public:
  FrontendWithFiles()
  {
  }

protected:
  virtual void SetUp()
  {
    fs::create_directory(sandbox);
  }
  virtual void TearDown()
  {
    fs::remove_all(sandbox);
  }

  const fs::path sandbox{"/tmp/sandbox_json_backend"};

};


//TODO: why is this in the backend testing... oh well...
TEST_F(FrontendWithFiles, testLoadingFrontendWithJson) {
    using namespace dyno;
    auto scenario = dyno_testing::makeDefaultScenario();

    std::map<FrameId, RGBDInstanceOutputPacket::Ptr> rgbd_output;

    for(size_t i = 0; i < 10; i++) {
        auto output = scenario.getOutput(i);
        rgbd_output.insert({i, output});
    }

    fs::path tmp_bison_path = sandbox / "simple_bison.bson";
    std::string tmp_bison_path_str = tmp_bison_path;

    JsonConverter::WriteOutJson(rgbd_output, tmp_bison_path_str, JsonConverter::Format::BSON);

    FrontendOfflinePipeline<RGBDBackendModule::ModuleTraits> offline_backed("offline-rgbdfrontend", tmp_bison_path_str);

    ThreadsafeQueue<FrontendOutputPacketBase::ConstPtr> queue;
    offline_backed.registerOutputQueue(&queue);

     int consumed = 0;
    while(offline_backed.spinOnce()) {
        FrontendOutputPacketBase::ConstPtr base;
        bool result = queue.popBlocking(base);

        EXPECT_TRUE(result);
        EXPECT_TRUE(base != nullptr);
        RGBDInstanceOutputPacket::ConstPtr derived = safeCast<FrontendOutputPacketBase, RGBDInstanceOutputPacket>(base);
        EXPECT_TRUE(derived != nullptr);

        //this value indexed by consume will be wrong if pop blocking doesnt work
        EXPECT_EQ(*derived, *rgbd_output.at(consumed));
        consumed++;
    }

    //should spin until the pipeline is shutdown which happens when we dont have any more data
    offline_backed.spin();
    EXPECT_TRUE(offline_backed.isShutdown());
    EXPECT_EQ(consumed, 10); //we should habve processed data 10 tiuems

}



//https://www.cppstories.com/2018/09/visit-variants/
template<class... Ts> struct overload : Ts... { using Ts::operator()...; };
template<class... Ts> overload(Ts...) -> overload<Ts...>; // line not needed in C++20


// //must be T& not const T&
// template<typename T>
// struct Visitor {

//     void process(const T& t) {
//         LOG(INFO) << t;
//     }

//     void operator()(T& t) { process(t);}

// };



// template<typename T> struct is_variant : std::false_type {};

// template<typename ...Variants>
// struct is_variant<std::variant<Variants...>> : std::true_type {};

// template<typename T>
// inline constexpr bool is_variant_v=is_variant<T>::value;

// // main lookup logic of looking up a type in a list.
// //https://www.appsloveworld.com/cplus/100/22/how-do-i-check-if-an-stdvariant-can-hold-a-certain-type
// template<typename T, typename... Variants>
// struct isoneof : public std::false_type {};

// template<typename T, typename FrontVariant, typename... RestVariants>
// struct isoneof<T, FrontVariant, RestVariants...> : public
//   std::conditional<
//     std::is_same<T, FrontVariant>::value,
//     std::true_type,
//     isoneof<T, RestVariants...>
//   >::type {};

// // convenience wrapper for std::variant<>.
// template<typename T, typename Variants>
// struct isvariantmember : public std::false_type {};

// template<typename T, typename... Variants>
// struct isvariantmember<T, std::variant<Variants...>> : public isoneof<T, Variants...> {};

// template<typename T, typename... Variants>
// inline constexpr bool isvariantmember_v = isvariantmember<T, Variants...>::value;


// template<typename IPacket, typename OPacket, typename MInput = IPacket, typename MOutput = OPacket>
// class MBase {
// public:
//     using InputPacket = IPacket;
//     using OutputPacket = OPacket;
//     using ModuleInput = MInput;
//     using ModuleOutput = MOutput;

// public:
//     constexpr static bool IsInputPacketVariant = is_variant_v<IPacket>;
//     constexpr static bool IsOutputPacketVariant = is_variant_v<OutputPacket>;

//     constexpr static bool IsModuleInputVariant = is_variant_v<ModuleInput>;
//     constexpr static bool IsModuleOutputVariant = is_variant_v<ModuleOutput>;


//     //either InputPacket is a variant or the InputPacket and ModuleInput are the same
//     //and no casting is required
//     static_assert(IsInputPacketVariant || std::is_same_v<ModuleInput,InputPacket>,
//         "InputPacket specified by IPacket is not a std::variant or ModuleInput and InputPacket are not the same type");

//     static_assert(IsOutputPacketVariant || std::is_same_v<ModuleOutput,OutputPacket>,
//         "OutputPacket specified by OPacket is not a std::variant or ModuleOutput and OutputPacket are not the same type");

//     // //if IsInputPacketVariant, then ModuleInput cannot also be a variant!!
//     static_assert(!(IsInputPacketVariant && IsModuleInputVariant),
//         "InputPacket is a variant, so ModuleInput cannot also be a variant, but a type within the variant");

//     static_assert(!(IsOutputPacketVariant && IsModuleOutputVariant),
//         "OutputPacket is a variant, so ModuleOutput cannot also be a variant, but a type within the variant");

//     static_assert(isvariantmember_v<ModuleInput, InputPacket> || std::is_same_v<ModuleInput,InputPacket>,
//         "If InputPacket is a variant, ModuleInput is not a type within the variant, or ModuleInput and InputPacket types are not the same");

//     static_assert(isvariantmember_v<ModuleOutput, OutputPacket> || std::is_same_v<ModuleOutput,OutputPacket>,
//         "If InputPacket is a variant, ModuleOutput is not a type within the variant, or ModuleOutput and InputPacket types are not the same");

// public:
//     ModuleInput cast(const InputPacket& packet) const {
//         if constexpr (IsInputPacketVariant) {
//             try {
//                 //packet should be a variant the ModuleOutput is a type within the std::variant
//                 //and the static asserts guarantee that ModuleInput is also not a variant
//                 return std::get<ModuleInput>(packet);
//             }
//             catch(const std::bad_variant_access& e) {
//                 throw std::runtime_error("Bad access");
//             }
//         }
//         else {
//            //if InputPacket is not a variant, then the static asserts guarantee that InputPacket == ModuleInput;
//            return packet;
//         }
//     }




//     virtual ModuleOutput process(const ModuleInput& input) {
//         LOG(INFO) << input;
//         return ModuleOutput{};
//     }

//     OutputPacket spinOnce(const InputPacket& packet) {
//         return process(cast(packet));
//     }

// };


// using VI = std::variant<double, std::string>;

// template<typename INPUT, typename OUTPUT>
// class Module : public MBase<VI, VI, INPUT, OUTPUT> {
// public:
//     using MBase<VI, VI, INPUT, OUTPUT>::process;
// };

// class DerivedModule : public Module<double, std::string> {

// public:
//     std::string process(const double& input) override {
//         LOG(INFO) << "Input" << input;
//         return "result";
//     }
// };



// TEST(PipelineBase, checkCompilation) {
//     using VariantInput = std::variant<double, int>;
//     //IPacket is variant so we need to specify ModuleInput
//     MBase<VariantInput, double, double>{};
//     MBase<int, double, int, double>{};
//     MBase<VariantInput, double, int, double>{};
//     // MBase<VariantInput, double, int, std::string>{};
//     // MBase<VariantInput, double, VariantInput, double>{};

//     // constexpr bool r = MBase<VariantInput, double, int, double>::IsModuleInputVariantMember;
// }

// TEST(PipelineBase, moduleProcess) {
//     // using VariantInput = std::variant<double, std::string>;
//     // //IPacket is variant so we need to specify ModuleInput
//     // MBase<VariantInput, double, std::string> mb{};

//     // //this should cast the input to a double
//     // mb.spinOnce("hi");

//     DerivedModule dm;
//     dm.spinOnce("string");
// }


// TEST(PipelineBase, testWithVairant) {

//     using Var = std::variant<int, std::string>;
//     using VarPipeline = FunctionalSIMOPipelineModule<Var, NullPipelinePayload>;

//     using VarModule = VariantModule<Var, NullPipelinePayload, std::string>;

//     VarPipeline::InputQueue input_queue;
//     VarPipeline::OutputQueue output_queue;
//     VarModule var_module;
//     // Visitor<std::string> string_visitor;
//     // Visitor<int> int_visitor;

//     VarPipeline p("var_module", &input_queue,
//         [&](const VarPipeline::InputConstSharedPtr& var_ptr) -> VarPipeline::OutputConstSharedPtr {
//             // return var_module.spinOnce(var_ptr);
//             return var_module.spinOnce(var_ptr);


//         },
//         false);

//     // input_queue.push(std::make_shared<Var>((int)3));
//     input_queue.push(std::make_shared<Var>("string"));

//     p.spinOnce();
//     p.spinOnce();


// }
```

## File: thread_safety/test_threadsafe_imu_buffer.cc
```
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

/*******************************************************************************
 * MIT License
 *
 * Copyright (c) 2019 Toni Rosinol
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 *all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *******************************************************************************/

/********************************************************************************
 Copyright 2017 Autonomous Systems Lab, ETH Zurich, Switzerland
   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at
       http://www.apache.org/licenses/LICENSE-2.0
   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
*********************************************************************************/

#include "dynosam/frontend/imu/ThreadSafeImuBuffer.hpp"
#include "dynosam/frontend/imu/Imu-Definitions.hpp"
#include "dynosam/frontend/imu/ImuMeasurements.hpp"

#include <glog/logging.h>
#include <gtest/gtest.h>

namespace dyno {

TEST(ThreadsafeImuBuffer, PopFromEmptyBuffer) {
  dyno::ThreadsafeImuBuffer buffer(-1);
  // Pop from empty buffer.
  Timestamps imu_timestamps(1, 2);
  ImuAccGyrs imu_measurements(6, 2);
  {
    dyno::ThreadsafeImuBuffer::QueryResult success =
        buffer.getImuDataBtwTimestamps(50, 100, &imu_timestamps,
                                       &imu_measurements);
    EXPECT_EQ(success,
              ThreadsafeImuBuffer::QueryResult::kDataNotYetAvailable);
    EXPECT_EQ(0u, imu_timestamps.size());
    EXPECT_EQ(0u, imu_measurements.size());
  }
  {
    dyno::ThreadsafeImuBuffer::QueryResult success =
        buffer.getImuDataBtwTimestamps(50, 100, &imu_timestamps,
                                       &imu_measurements, true);
    EXPECT_EQ(success,
              ThreadsafeImuBuffer::QueryResult::kDataNotYetAvailable);
    EXPECT_EQ(0u, imu_timestamps.size());
    EXPECT_EQ(0u, imu_measurements.size());
  }
  {
    dyno::ThreadsafeImuBuffer::QueryResult success =
        buffer.getImuDataInterpolatedUpperBorder(50, 100, &imu_timestamps,
                                                 &imu_measurements);
    EXPECT_EQ(success,
              ThreadsafeImuBuffer::QueryResult::kDataNotYetAvailable);
    EXPECT_EQ(0u, imu_timestamps.size());
    EXPECT_EQ(0u, imu_measurements.size());
  }
  {
    dyno::ThreadsafeImuBuffer::QueryResult success =
        buffer.getImuDataInterpolatedBorders(50, 100, &imu_timestamps,
                                             &imu_measurements);
    EXPECT_EQ(success,
              ThreadsafeImuBuffer::QueryResult::kDataNotYetAvailable);
    EXPECT_EQ(0u, imu_timestamps.size());
    EXPECT_EQ(0u, imu_measurements.size());
  }
}

TEST(ThreadsafeImuBuffer, LinearInterpolate) {
  ImuAccGyr y;
  dyno::ThreadsafeImuBuffer::linearInterpolate(
      10, ImuAccGyr::Constant(10.0), 20, ImuAccGyr::Constant(50.0), 15, &y);
  EXPECT_EQ(y, ImuAccGyr::Constant(30.0));
}

TEST(ThreadsafeImuBuffer, getImuDataBtwTimestamps) {
  dyno::ThreadsafeImuBuffer buffer(-1);
  buffer.addMeasurement(10, ImuAccGyr::Constant(10.0));
  buffer.addMeasurement(15, ImuAccGyr::Constant(15.0));
  buffer.addMeasurement(20, ImuAccGyr::Constant(20.0));
  buffer.addMeasurement(25, ImuAccGyr::Constant(25.0));
  buffer.addMeasurement(30, ImuAccGyr::Constant(30.0));
  buffer.addMeasurement(40, ImuAccGyr::Constant(40.0));
  buffer.addMeasurement(50, ImuAccGyr::Constant(50.0));

  Timestamps imu_timestamps;
  ImuAccGyrs imu_measurements;
  dyno::ThreadsafeImuBuffer::QueryResult result;

  // Test aligned getter.
  result = buffer.getImuDataBtwTimestamps(20, 30, &imu_timestamps,
                                          &imu_measurements);
  EXPECT_EQ(result,
            dyno::ThreadsafeImuBuffer::QueryResult::kDataAvailable);
  EXPECT_EQ(imu_timestamps.cols(), 1);
  EXPECT_EQ(imu_measurements.cols(), 1);
  EXPECT_EQ(imu_timestamps(0), 25);
  EXPECT_EQ(imu_measurements.col(0)(0), 25.0);

  // Test aligned getter, but asking for lower bound
  result = buffer.getImuDataBtwTimestamps(20, 30, &imu_timestamps,
                                          &imu_measurements, true);
  EXPECT_EQ(result,
            dyno::ThreadsafeImuBuffer::QueryResult::kDataAvailable);
  EXPECT_EQ(imu_timestamps.cols(), 2);
  EXPECT_EQ(imu_measurements.cols(), 2);
  EXPECT_EQ(imu_timestamps(0), 20);
  EXPECT_EQ(imu_timestamps(1), 25);
  EXPECT_EQ(imu_measurements.col(0)(0), 20.0);
  EXPECT_EQ(imu_measurements.col(1)(0), 25.0);

  // Test unaligned getter (no lower/upper-interpolation).
  result = buffer.getImuDataBtwTimestamps(19, 31, &imu_timestamps,
                                          &imu_measurements);
  EXPECT_EQ(result,
            dyno::ThreadsafeImuBuffer::QueryResult::kDataAvailable);
  EXPECT_EQ(imu_timestamps.cols(), 3);
  EXPECT_EQ(imu_measurements.cols(), 3);
  EXPECT_EQ(imu_timestamps(0), 20);
  EXPECT_EQ(imu_timestamps(1), 25);
  EXPECT_EQ(imu_timestamps(2), 30);
  EXPECT_EQ(imu_measurements.col(0)(0), 20.0);
  EXPECT_EQ(imu_measurements.col(1)(0), 25.0);
  EXPECT_EQ(imu_measurements.col(2)(0), 30.0);

  // Test unaligned getter but asking for lower bound.
  result = buffer.getImuDataBtwTimestamps(19, 31, &imu_timestamps,
                                          &imu_measurements, true);
  EXPECT_EQ(result,
            dyno::ThreadsafeImuBuffer::QueryResult::kDataAvailable);
  EXPECT_EQ(imu_timestamps.cols(), 3);
  EXPECT_EQ(imu_measurements.cols(), 3);
  EXPECT_EQ(imu_timestamps(0), 20);
  EXPECT_EQ(imu_timestamps(1), 25);
  EXPECT_EQ(imu_timestamps(2), 30);
  EXPECT_EQ(imu_measurements.col(0)(0), 20.0);
  EXPECT_EQ(imu_measurements.col(1)(0), 25.0);
  EXPECT_EQ(imu_measurements.col(2)(0), 30.0);

  // Fail: query out of upper bound.
  result = buffer.getImuDataBtwTimestamps(40, 51, &imu_timestamps,
                                          &imu_measurements);
  EXPECT_EQ(result,
            dyno::ThreadsafeImuBuffer::QueryResult::kDataNotYetAvailable);
  result = buffer.getImuDataBtwTimestamps(60, 61, &imu_timestamps,
                                          &imu_measurements);
  EXPECT_EQ(result,
            dyno::ThreadsafeImuBuffer::QueryResult::kDataNotYetAvailable);

  // Fail: query out of lower bound.
  result = buffer.getImuDataBtwTimestamps(-1, 20, &imu_timestamps,
                                          &imu_measurements);
  EXPECT_EQ(result,
            dyno::ThreadsafeImuBuffer::QueryResult::kDataNeverAvailable);
  result = buffer.getImuDataBtwTimestamps(-20, -10, &imu_timestamps,
                                          &imu_measurements);
  EXPECT_EQ(result,
            dyno::ThreadsafeImuBuffer::QueryResult::kDataNeverAvailable);

  // Query in between two values: return nothing.
  result = buffer.getImuDataBtwTimestamps(21, 24, &imu_timestamps,
                                          &imu_measurements);
  EXPECT_EQ(result, dyno::ThreadsafeImuBuffer::QueryResult::
                        kTooFewMeasurementsAvailable);
  EXPECT_EQ(imu_timestamps.cols(), 0);
  EXPECT_EQ(imu_measurements.cols(), 0);
  result = buffer.getImuDataBtwTimestamps(21, 24, &imu_timestamps,
                                          &imu_measurements, true);
  EXPECT_EQ(result, dyno::ThreadsafeImuBuffer::QueryResult::
                        kTooFewMeasurementsAvailable);
  EXPECT_EQ(imu_timestamps.cols(), 0);
  EXPECT_EQ(imu_measurements.cols(), 0);

  // Query right between two values: return nothing.
  result = buffer.getImuDataBtwTimestamps(20, 25, &imu_timestamps,
                                          &imu_measurements);
  EXPECT_EQ(result, dyno::ThreadsafeImuBuffer::QueryResult::
                        kTooFewMeasurementsAvailable);
  EXPECT_EQ(imu_timestamps.cols(), 0);
  EXPECT_EQ(imu_measurements.cols(), 0);

  // Query right between two values but ask for lower bound: return lower bound.
  result = buffer.getImuDataBtwTimestamps(20, 25, &imu_timestamps,
                                          &imu_measurements, true);
  EXPECT_EQ(result,
            dyno::ThreadsafeImuBuffer::QueryResult::kDataAvailable);
  EXPECT_EQ(imu_timestamps.cols(), 1);
  EXPECT_EQ(imu_measurements.cols(), 1);
  EXPECT_EQ(imu_timestamps(0), 20);
  EXPECT_EQ(imu_measurements.col(0)(0), 20.0);
}

TEST(ThreadsafeImuBuffer, getImuDataInterpolatedBorders) {
  dyno::ThreadsafeImuBuffer buffer(-1);
  buffer.addMeasurement(10, ImuAccGyr::Constant(10.0));
  buffer.addMeasurement(15, ImuAccGyr::Constant(15.0));
  buffer.addMeasurement(20, ImuAccGyr::Constant(20.0));
  buffer.addMeasurement(25, ImuAccGyr::Constant(25.0));
  buffer.addMeasurement(30, ImuAccGyr::Constant(30.0));
  buffer.addMeasurement(40, ImuAccGyr::Constant(40.0));
  buffer.addMeasurement(50, ImuAccGyr::Constant(50.0));

  Timestamps imu_timestamps;
  ImuAccGyrs imu_measurements;
  dyno::ThreadsafeImuBuffer::QueryResult result;

  // Test aligned getter (no-interpolation, only border values).
  result = buffer.getImuDataInterpolatedBorders(20, 30, &imu_timestamps,
                                                &imu_measurements);
  EXPECT_EQ(result,
            dyno::ThreadsafeImuBuffer::QueryResult::kDataAvailable);
  EXPECT_EQ(imu_timestamps.cols(), 3);
  EXPECT_EQ(imu_measurements.cols(), 3);
  EXPECT_EQ(imu_timestamps(0), 20);
  EXPECT_EQ(imu_timestamps(1), 25);
  EXPECT_EQ(imu_timestamps(2), 30);
  EXPECT_EQ(imu_measurements.col(0)(0), 20.0);
  EXPECT_EQ(imu_measurements.col(1)(0), 25.0);
  EXPECT_EQ(imu_measurements.col(2)(0), 30.0);

  // Test aligned getter (no-interpolation).
  result = buffer.getImuDataInterpolatedBorders(20, 40, &imu_timestamps,
                                                &imu_measurements);
  EXPECT_EQ(result,
            dyno::ThreadsafeImuBuffer::QueryResult::kDataAvailable);
  EXPECT_EQ(imu_timestamps.cols(), 4);
  EXPECT_EQ(imu_measurements.cols(), 4);
  EXPECT_EQ(imu_timestamps(0), 20);
  EXPECT_EQ(imu_timestamps(1), 25);
  EXPECT_EQ(imu_timestamps(2), 30);
  EXPECT_EQ(imu_timestamps(3), 40);
  EXPECT_EQ(imu_measurements.col(0)(0), 20.0);
  EXPECT_EQ(imu_measurements.col(1)(0), 25.0);
  EXPECT_EQ(imu_measurements.col(2)(0), 30.0);
  EXPECT_EQ(imu_measurements.col(3)(0), 40.0);

  // Test unaligned getter (lower/upper-interpolation).
  result = buffer.getImuDataInterpolatedBorders(19, 21, &imu_timestamps,
                                                &imu_measurements);
  EXPECT_EQ(result,
            dyno::ThreadsafeImuBuffer::QueryResult::kDataAvailable);
  EXPECT_EQ(imu_timestamps.cols(), 3);
  EXPECT_EQ(imu_measurements.cols(), 3);
  EXPECT_EQ(imu_timestamps(0), 19);
  EXPECT_EQ(imu_timestamps(1), 20);
  EXPECT_EQ(imu_timestamps(2), 21);
  EXPECT_EQ(imu_measurements.col(0)(0), 19.0);
  EXPECT_EQ(imu_measurements.col(1)(0), 20.0);
  EXPECT_EQ(imu_measurements.col(2)(0), 21.0);

  // Fail: query out of upper bound.
  result = buffer.getImuDataInterpolatedBorders(40, 51, &imu_timestamps,
                                                &imu_measurements);
  EXPECT_EQ(result,
            dyno::ThreadsafeImuBuffer::QueryResult::kDataNotYetAvailable);
  result = buffer.getImuDataInterpolatedBorders(60, 61, &imu_timestamps,
                                                &imu_measurements);
  EXPECT_EQ(result,
            dyno::ThreadsafeImuBuffer::QueryResult::kDataNotYetAvailable);

  // Fail: query out of lower bound.
  result = buffer.getImuDataInterpolatedBorders(-1, 20, &imu_timestamps,
                                                &imu_measurements);
  EXPECT_EQ(result,
            dyno::ThreadsafeImuBuffer::QueryResult::kDataNeverAvailable);
  result = buffer.getImuDataInterpolatedBorders(-20, -10, &imu_timestamps,
                                                &imu_measurements);
  EXPECT_EQ(result,
            dyno::ThreadsafeImuBuffer::QueryResult::kDataNeverAvailable);

  // Query between two values: return the border values.
  result = buffer.getImuDataInterpolatedBorders(21, 29, &imu_timestamps,
                                                &imu_measurements);
  EXPECT_EQ(result,
            dyno::ThreadsafeImuBuffer::QueryResult::kDataAvailable);
  EXPECT_EQ(imu_timestamps.cols(), 3);
  EXPECT_EQ(imu_measurements.cols(), 3);
  EXPECT_EQ(imu_timestamps(0), 21);
  EXPECT_EQ(imu_timestamps(1), 25);
  EXPECT_EQ(imu_timestamps(2), 29);
  EXPECT_EQ(imu_measurements.col(0)(0), 21.0);
  EXPECT_EQ(imu_measurements.col(1)(0), 25.0);
  EXPECT_EQ(imu_measurements.col(2)(0), 29.0);
}

TEST(ThreadsafeImuBuffer, getImuDataInterpolatedUpperBorder) {
  dyno::ThreadsafeImuBuffer buffer(-1);
  buffer.addMeasurement(10, ImuAccGyr::Constant(10.0));
  buffer.addMeasurement(15, ImuAccGyr::Constant(15.0));
  buffer.addMeasurement(20, ImuAccGyr::Constant(20.0));
  buffer.addMeasurement(25, ImuAccGyr::Constant(25.0));
  buffer.addMeasurement(30, ImuAccGyr::Constant(30.0));
  buffer.addMeasurement(40, ImuAccGyr::Constant(40.0));
  buffer.addMeasurement(50, ImuAccGyr::Constant(50.0));

  Timestamps imu_timestamps;
  ImuAccGyrs imu_measurements;
  dyno::ThreadsafeImuBuffer::QueryResult result;

  // Test aligned getter (no-interpolation).
  result = buffer.getImuDataInterpolatedUpperBorder(20, 40, &imu_timestamps,
                                                    &imu_measurements);
  EXPECT_EQ(result,
            dyno::ThreadsafeImuBuffer::QueryResult::kDataAvailable);
  EXPECT_EQ(imu_timestamps.cols(), 4);
  EXPECT_EQ(imu_measurements.cols(), 4);
  EXPECT_EQ(imu_timestamps(0), 20);
  EXPECT_EQ(imu_timestamps(1), 25);
  EXPECT_EQ(imu_timestamps(2), 30);
  EXPECT_EQ(imu_timestamps(3), 40);
  EXPECT_EQ(imu_measurements.col(0)(0), 20.0);
  EXPECT_EQ(imu_measurements.col(1)(0), 25.0);
  EXPECT_EQ(imu_measurements.col(2)(0), 30.0);
  EXPECT_EQ(imu_measurements.col(3)(0), 40.0);

  // Test unaligned getter (only upper-interpolation).
  result = buffer.getImuDataInterpolatedUpperBorder(19, 21, &imu_timestamps,
                                                    &imu_measurements);
  EXPECT_EQ(result,
            dyno::ThreadsafeImuBuffer::QueryResult::kDataAvailable);
  EXPECT_EQ(imu_timestamps.cols(), 2);
  EXPECT_EQ(imu_measurements.cols(), 2);
  EXPECT_EQ(imu_timestamps(0), 20);
  EXPECT_EQ(imu_timestamps(1), 21);
  EXPECT_EQ(imu_measurements.col(0)(0), 20.0);
  EXPECT_EQ(imu_measurements.col(1)(0), 21.0);

  // Fail: query out of upper bound.
  result = buffer.getImuDataInterpolatedUpperBorder(40, 51, &imu_timestamps,
                                                    &imu_measurements);
  EXPECT_EQ(result,
            dyno::ThreadsafeImuBuffer::QueryResult::kDataNotYetAvailable);
  result = buffer.getImuDataInterpolatedUpperBorder(60, 61, &imu_timestamps,
                                                    &imu_measurements);
  EXPECT_EQ(result,
            dyno::ThreadsafeImuBuffer::QueryResult::kDataNotYetAvailable);

  // Fail: query out of lower bound.
  result = buffer.getImuDataInterpolatedUpperBorder(9, 20, &imu_timestamps,
                                                    &imu_measurements);
  EXPECT_EQ(result,
            dyno::ThreadsafeImuBuffer::QueryResult::kDataNeverAvailable);
  result = buffer.getImuDataInterpolatedUpperBorder(-20, -10, &imu_timestamps,
                                                    &imu_measurements);
  EXPECT_EQ(result,
            dyno::ThreadsafeImuBuffer::QueryResult::kDataNeverAvailable);

  // Query in between two values: return too few measurements available.
  // even if asked to interpolate, there are no measurements in between
  // given timestamps.
  result = buffer.getImuDataInterpolatedUpperBorder(21, 24, &imu_timestamps,
                                                    &imu_measurements);
  EXPECT_EQ(result, dyno::ThreadsafeImuBuffer::QueryResult::
                        kTooFewMeasurementsAvailable);
  EXPECT_EQ(imu_timestamps.cols(), 0);
  EXPECT_EQ(imu_measurements.cols(), 0);

  // Query with only one value inside interval:
  // return the interpolated border value for upper border only,
  // and one measurement
  result = buffer.getImuDataInterpolatedUpperBorder(21, 29, &imu_timestamps,
                                                    &imu_measurements);
  EXPECT_EQ(result,
            dyno::ThreadsafeImuBuffer::QueryResult::kDataAvailable);
  EXPECT_EQ(imu_timestamps.cols(), 2);
  EXPECT_EQ(imu_measurements.cols(), 2);
  EXPECT_EQ(imu_timestamps(0), 25);
  EXPECT_EQ(imu_timestamps(1), 29);
  EXPECT_EQ(imu_measurements.col(0)(0), 25.0);
  EXPECT_EQ(imu_measurements.col(1)(0), 29.0);
}

//// Disabled because CppUnitTests does not support Expect DEATH.
// TEST(ThreadsafeImuBuffer, DeathOnAddDataNotIncreasingTimestamp) {
//  dyno::ThreadsafeImuBuffer buffer(-1);
//
//  ImuAccGyr imu_measurement;
//  imu_measurement.setRandom();
//
//  buffer.addMeasurement(0u, imu_measurement);
//  buffer.addMeasurement(10u, imu_measurement);
//  //EXPECT_DEATH(buffer.addMeasurement(9u, imu_measurement), "^");
//}

TEST(ThreadsafeImuBuffer, TestAddMeasurements) {
  const size_t kNumMeasurements = 10;
  dyno::ThreadsafeImuBuffer buffer(-1);

  // Create IMU measurements and fill buffer.
  Timestamps imu_timestamps_groundtruth(1, kNumMeasurements);
  ImuAccGyrs imu_measurements_groundtruth(6, kNumMeasurements);

  for (size_t idx = 0; idx < kNumMeasurements; ++idx) {
    Timestamp timestamp = static_cast<Timestamp>(idx * 4.3);
    ImuAccGyr imu_measurement;
    imu_measurement.setConstant(idx);
    imu_timestamps_groundtruth(idx) = timestamp;
    imu_measurements_groundtruth.col(idx) = imu_measurement;
  }
  buffer.addMeasurements(imu_timestamps_groundtruth,
                         imu_measurements_groundtruth);
}

}  // namespace dyno
```

## File: thread_safety/thread_safe_queue_tests.cc
```
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

#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <glog/logging.h>
#include <gtest/gtest.h>

#include "dynosam/pipeline/ThreadSafeQueue.hpp"


void consumer(ThreadsafeQueue<std::string>& q,  // NOLINT
              const std::atomic_bool& kill_switch)
{
  while (!kill_switch)
  {
    std::string msg = "No msg!";
    if (q.popBlocking(msg))
    {
      VLOG(1) << "Got msg: " << msg << '\n';
    }
  }
  q.shutdown();
}

void producer_milliseconds(ThreadsafeQueue<std::string>& q,  // NOLINT
                           const std::atomic_bool& kill_switch, int delay)
{
  while (!kill_switch)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(delay));
    q.push("Hello World!");
  }
  q.shutdown();
}

void producer(ThreadsafeQueue<std::string>& q,  // NOLINT
              const std::atomic_bool& kill_switch)
{
  while (!kill_switch)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    q.push("Hello World!");
  }
  q.shutdown();
}

void blockingProducer(ThreadsafeQueue<std::string>& q,  // NOLINT
                      const std::atomic_bool& kill_switch)
{
  while (!kill_switch)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    q.pushBlockingIfFull("Hello World!", 5);
  }
  q.shutdown();
}

/* ************************************************************************* */
TEST(testThreadsafeQueue, popBlocking_by_reference)
{
  ThreadsafeQueue<std::string> q;
  std::thread p([&] {
    q.push("Hello World!");
    q.push("Hello World 2!");
  });
  std::string s;
  q.popBlocking(s);
  EXPECT_EQ(s, "Hello World!");
  q.popBlocking(s);
  EXPECT_EQ(s, "Hello World 2!");
  q.shutdown();
  EXPECT_FALSE(q.popBlocking(s));
  EXPECT_EQ(s, "Hello World 2!");

  // Leave some time for p to finish its work.
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  EXPECT_TRUE(p.joinable());
  p.join();
}

/* ************************************************************************* */
TEST(testThreadsafeQueue, popBlocking_by_shared_ptr)
{
  ThreadsafeQueue<std::string> q;
  std::thread p([&] {
    q.push("Hello World!");
    q.push("Hello World 2!");
  });
  std::shared_ptr<std::string> s = q.popBlocking();
  EXPECT_EQ(*s, "Hello World!");
  auto s2 = q.popBlocking();
  EXPECT_EQ(*s2, "Hello World 2!");
  q.shutdown();
  EXPECT_EQ(q.popBlocking(), nullptr);

  // Leave some time for p to finish its work.
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  EXPECT_TRUE(p.joinable());
  p.join();
}

/* ************************************************************************* */
TEST(testThreadsafeQueue, push)
{
  ThreadsafeQueue<std::string> q;
  std::thread p([&] {
    q.push(std::string("Hello World!"));
    std::string s = "Hello World 2!";
    q.push(s);
  });
  std::shared_ptr<std::string> s = q.popBlocking();
  EXPECT_EQ(*s, "Hello World!");
  auto s2 = q.popBlocking();
  EXPECT_EQ(*s2, "Hello World 2!");
  q.shutdown();
  EXPECT_EQ(q.popBlocking(), nullptr);

  // Leave some time for p to finish its work.
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  EXPECT_TRUE(p.joinable());
  p.join();
}

TEST(testThreadsafeQueue, pointerContainer)
{
  ThreadsafeQueue<std::shared_ptr<std::string>> q;
  std::thread p([&] {
    q.push(std::make_shared<std::string>("Hello World!"));
    auto s = std::make_shared<std::string>("Hello World 2!");
    q.push(s);
  });

  // wow this is gross...
  std::shared_ptr<std::shared_ptr<std::string>> s = q.popBlocking();
  EXPECT_EQ(**s, "Hello World!");
  auto s2 = q.popBlocking();
  EXPECT_EQ(**s2, "Hello World 2!");
  q.shutdown();
  EXPECT_EQ(q.popBlocking(), nullptr);

  // Leave some time for p to finish its work.
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  EXPECT_TRUE(p.joinable());
  p.join();
}

/* ************************************************************************* */
TEST(testThreadsafeQueue, pushBlockingIfFull)
{
  // Here we test only its nominal push behavior, not the blocking behavior
  ThreadsafeQueue<std::string> q;
  std::thread p([&] {
    q.pushBlockingIfFull(std::string("Hello World!"), 2);
    std::string s = "Hello World 2!";
    q.pushBlockingIfFull(s, 2);
  });
  std::shared_ptr<std::string> s = q.popBlocking();
  EXPECT_EQ(*s, "Hello World!");
  auto s2 = q.popBlocking();
  EXPECT_EQ(*s2, "Hello World 2!");
  q.shutdown();
  EXPECT_EQ(q.popBlocking(), nullptr);

  // Leave some time for p to finish its work.
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  EXPECT_TRUE(p.joinable());
  p.join();
}

/* ************************************************************************* */
TEST(testThreadsafeQueue, producer_consumer)
{
  ThreadsafeQueue<std::string> q;
  std::atomic_bool kill_switch(false);
  std::thread c(consumer, std::ref(q), std::ref(kill_switch));
  std::thread p(producer, std::ref(q), std::ref(kill_switch));

  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  VLOG(1) << "Shutdown queue.\n";
  q.shutdown();

  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  VLOG(1) << "Resume queue.\n";
  q.resume();

  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  VLOG(1) << "Joining threads.\n";
  kill_switch = true;
  c.join();
  p.join();
  VLOG(1) << "Threads joined.\n";
}

/* ************************************************************************* */
TEST(testThreadsafeQueue, blocking_producer)
{
  ThreadsafeQueue<std::string> q;
  std::atomic_bool kill_switch(false);
  std::thread p(blockingProducer, std::ref(q), std::ref(kill_switch));

  // Give plenty of time to the blockingProducer to fill-in completely the queue
  // and be blocked.
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  VLOG(1) << "Shutdown queue.\n";
  q.shutdown();

  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  VLOG(1) << "Resume queue.\n";
  q.resume();

  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  VLOG(1) << "Joining thread.\n";
  q.shutdown();
  kill_switch = true;
  p.join();
  VLOG(1) << "Thread joined.\n";

  // Need to resume the queue to be able to pop...
  q.resume();

  // Expect non-empty queue.
  EXPECT_TRUE(!q.empty());
  size_t queue_size = 0;
  while (!q.empty())
  {
    std::string output;
    EXPECT_TRUE(q.pop(output));
    EXPECT_EQ(output, "Hello World!");
    ++queue_size;
  }
  // Expect the size of the queue to be the maximum size of the queue
  EXPECT_EQ(queue_size, 5u);
}

/* ************************************************************************* */
TEST(testThreadsafeQueue, stress_test)
{
  ThreadsafeQueue<std::string> q;
  std::atomic_bool kill_switch(false);
  std::vector<std::thread> cs;
  for (size_t i = 0; i < 10; i++)
  {
    // Create 10 consumers.
    cs.push_back(std::thread(consumer, std::ref(q), std::ref(kill_switch)));
  }
  std::vector<std::thread> ps;
  for (size_t i = 0; i < 10; i++)
  {
    // Create 10 producers.
    ps.push_back(std::thread(producer, std::ref(q), std::ref(kill_switch)));
  }
  std::vector<std::thread> blocking_ps;
  for (size_t i = 0; i < 10; i++)
  {
    // Create 10 producers.
    ps.push_back(std::thread(blockingProducer, std::ref(q), std::ref(kill_switch)));
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  VLOG(1) << "Shutdown queue.\n";
  q.shutdown();

  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  VLOG(1) << "Resume queue.\n";
  q.resume();

  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  VLOG(1) << "Joining threads.\n";
  kill_switch = true;
  for (size_t i = 0; i < cs.size(); i++)
  {
    cs[i].join();
  }
  for (size_t i = 0; i < ps.size(); i++)
  {
    ps[i].join();
  }
  for (size_t i = 0; i < blocking_ps.size(); i++)
  {
    blocking_ps[i].join();
  }
  VLOG(1) << "Threads joined.\n";
}

TEST(testThreadsafeQueue, limited_queue_size)
{
  ThreadsafeQueue<std::string> q(5);
  std::atomic_bool kill_switch(false);
  std::thread p(producer_milliseconds, std::ref(q), std::ref(kill_switch), 10);

  // Give plenty of time to the producer_milliseconds to fill-in completely the queue
  // and be blocked.
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  // Expect the size of the queue to be the maximum size of the queue
  EXPECT_EQ(q.size(), 5u);

  q.shutdown();
  kill_switch = true;
  p.join();
}
```

## File: thread_safety/thread_safe_temporal_buffer_test.cc
```
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

#include <chrono>
#include <thread>

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <gtest/gtest.h>

#include "dynosam/pipeline/ThreadSafeTemporalBuffer.hpp"

namespace dyno
{
struct TestData
{
  explicit TestData(Timestamp time) : timestamp(time)
  {
  }
  TestData() = default;

  Timestamp timestamp;
};

TEST(ThreadsafeTemporalBuffer, testEqualsOverloading)
{
  ThreadsafeTemporalBuffer<TestData> buffer_1(-1);
  ThreadsafeTemporalBuffer<TestData> buffer_2(100);

  EXPECT_EQ(buffer_1.bufferLength(), -1);
  EXPECT_EQ(buffer_2.bufferLength(), 100);

  buffer_1.addValue(0, TestData(0));
  buffer_1.addValue(1, TestData(1));
  buffer_1.addValue(2, TestData(2));

  EXPECT_EQ(buffer_1.size(), 3u);
  EXPECT_EQ(buffer_2.size(), 0u);

  buffer_2 = buffer_1;
  EXPECT_EQ(buffer_1.size(), 3u);
  EXPECT_EQ(buffer_2.size(), 3u);

  EXPECT_EQ(buffer_1.bufferLength(), -1);
  EXPECT_EQ(buffer_2.bufferLength(), -1);

  // change buffer1 and buffer 2 should not change
  buffer_1.addValue(3, TestData(3));
  EXPECT_EQ(buffer_1.size(), 4u);
  EXPECT_EQ(buffer_2.size(), 3u);
}

TEST(ThreadsafeTemporalBuffer, testLagSizeInfinite)
{
  ThreadsafeTemporalBuffer<TestData> buffer_(-1);
  buffer_.addValue(0, TestData(0));
  EXPECT_EQ(buffer_.size(), 1u);
  buffer_.addValue(1271839713, TestData(1271839713));
  EXPECT_EQ(buffer_.size(), 2u);
  buffer_.addValue(10000, TestData(10000));
  EXPECT_EQ(buffer_.size(), 3u);
}

TEST(ThreadsafeTemporalBuffer, testLagSize10)
{
  ThreadsafeTemporalBuffer<TestData> buffer_(10);
  buffer_.addValue(0, TestData(0));

  const double kMaxDelta = 0.01;
  TestData retrieved_item;
  // check that we can retrieve this value now so wer can be sure we cannot retrieve it later
  EXPECT_TRUE(buffer_.getNearestValueToTime(0, kMaxDelta, &retrieved_item));
  EXPECT_EQ(retrieved_item.timestamp, 0);

  EXPECT_EQ(buffer_.size(), 1u);
  buffer_.addValue(4.3, TestData(4.3));
  EXPECT_EQ(buffer_.size(), 2u);
  buffer_.addValue(9.9, TestData(9.9));
  EXPECT_EQ(buffer_.size(), 3u);

  // add time past the buffer -> this data should stay in the buffer but push out the 0th value
  buffer_.addValue(10.3, TestData(10.3));
  EXPECT_EQ(buffer_.size(), 3u);
  EXPECT_FALSE(buffer_.getNearestValueToTime(0, kMaxDelta, &retrieved_item));
  EXPECT_TRUE(buffer_.getNearestValueToTime(10.2999, kMaxDelta, &retrieved_item));
  EXPECT_EQ(retrieved_item.timestamp, 10.3);
}

class ThreadsafeTemporalBufferFixture : public ::testing::Test
{
public:
  ThreadsafeTemporalBufferFixture() : buffer_(kBufferLengthS)
  {
  }

protected:
  virtual void SetUp()
  {
  }
  virtual void TearDown()
  {
  }  //
  void addValue(const TestData& data)
  {
    buffer_.addValue(data.timestamp, data);
  }

  static constexpr Timestamp kBufferLengthS = 100;
  ThreadsafeTemporalBuffer<TestData> buffer_;
};

TEST_F(ThreadsafeTemporalBufferFixture, SizeEmptyClearWork)
{
  EXPECT_TRUE(buffer_.empty());
  EXPECT_EQ(buffer_.size(), 0u);

  addValue(TestData(10));
  addValue(TestData(20));
  EXPECT_TRUE(!buffer_.empty());
  EXPECT_EQ(buffer_.size(), 2u);

  buffer_.clear();
  EXPECT_TRUE(buffer_.empty());
  EXPECT_EQ(buffer_.size(), 0u);
}

TEST_F(ThreadsafeTemporalBufferFixture, GetValueAtTimeWorks)
{
  addValue(TestData(3.1));
  addValue(TestData(10));
  addValue(TestData(0.004));
  addValue(TestData(40.234));

  TestData retrieved_item;
  EXPECT_TRUE(buffer_.getValueAtTime(3.1, &retrieved_item));
  EXPECT_EQ(retrieved_item.timestamp, 3.1);

  EXPECT_TRUE(buffer_.getValueAtTime(0.004, &retrieved_item));
  EXPECT_EQ(retrieved_item.timestamp, 0.004);

  EXPECT_TRUE(!buffer_.getValueAtTime(40, &retrieved_item));

  EXPECT_TRUE(buffer_.getValueAtTime(3.1, &retrieved_item));
  EXPECT_EQ(retrieved_item.timestamp, 3.1);

  EXPECT_TRUE(buffer_.getValueAtTime(40.234, &retrieved_item));
  EXPECT_EQ(retrieved_item.timestamp, 40.234);
}

TEST_F(ThreadsafeTemporalBufferFixture, GetNearestValueToTimeWorks)
{
  addValue(TestData(3.00));
  addValue(TestData(1.004));
  addValue(TestData(6.32));
  addValue(TestData(34));

  TestData retrieved_item;
  EXPECT_TRUE(buffer_.getNearestValueToTime(3, &retrieved_item));
  EXPECT_EQ(retrieved_item.timestamp, 3);

  EXPECT_TRUE(buffer_.getNearestValueToTime(0, &retrieved_item));
  EXPECT_EQ(retrieved_item.timestamp, 1.004);

  EXPECT_TRUE(buffer_.getNearestValueToTime(16, &retrieved_item));
  EXPECT_EQ(retrieved_item.timestamp, 6.32);

  EXPECT_TRUE(buffer_.getNearestValueToTime(34.432421, &retrieved_item));
  EXPECT_EQ(retrieved_item.timestamp, 34);

  EXPECT_TRUE(buffer_.getNearestValueToTime(32, &retrieved_item));
  EXPECT_EQ(retrieved_item.timestamp, 34);

  EXPECT_TRUE(buffer_.getNearestValueToTime(1232, &retrieved_item));
  EXPECT_EQ(retrieved_item.timestamp, 34);
}

TEST_F(ThreadsafeTemporalBufferFixture, GetNearestValueToTimeMaxDeltaWorks)
{
  addValue(TestData(30));
  addValue(TestData(10));
  addValue(TestData(20));

  const double kMaxDelta = 5;

  TestData retrieved_item;
  EXPECT_TRUE(buffer_.getNearestValueToTime(10, kMaxDelta, &retrieved_item));
  EXPECT_EQ(retrieved_item.timestamp, 10);

  EXPECT_TRUE(!buffer_.getNearestValueToTime(0, kMaxDelta, &retrieved_item));

  EXPECT_TRUE(buffer_.getNearestValueToTime(9, kMaxDelta, &retrieved_item));
  EXPECT_EQ(retrieved_item.timestamp, 10);

  EXPECT_TRUE(buffer_.getNearestValueToTime(16, kMaxDelta, &retrieved_item));
  EXPECT_EQ(retrieved_item.timestamp, 20);

  EXPECT_TRUE(buffer_.getNearestValueToTime(26, kMaxDelta, &retrieved_item));
  EXPECT_EQ(retrieved_item.timestamp, 30);

  EXPECT_TRUE(buffer_.getNearestValueToTime(32, kMaxDelta, &retrieved_item));
  EXPECT_EQ(retrieved_item.timestamp, 30);

  EXPECT_TRUE(!buffer_.getNearestValueToTime(36, kMaxDelta, &retrieved_item));

  buffer_.clear();
  addValue(TestData(10));
  addValue(TestData(20));

  EXPECT_TRUE(buffer_.getNearestValueToTime(9, kMaxDelta, &retrieved_item));
  EXPECT_EQ(retrieved_item.timestamp, 10);

  EXPECT_TRUE(buffer_.getNearestValueToTime(12, kMaxDelta, &retrieved_item));
  EXPECT_EQ(retrieved_item.timestamp, 10);

  EXPECT_TRUE(buffer_.getNearestValueToTime(16, kMaxDelta, &retrieved_item));
  EXPECT_EQ(retrieved_item.timestamp, 20);

  EXPECT_TRUE(buffer_.getNearestValueToTime(22, kMaxDelta, &retrieved_item));
  EXPECT_EQ(retrieved_item.timestamp, 20);

  buffer_.clear();
  addValue(TestData(10));

  EXPECT_TRUE(buffer_.getNearestValueToTime(6, kMaxDelta, &retrieved_item));
  EXPECT_EQ(retrieved_item.timestamp, 10);

  EXPECT_TRUE(buffer_.getNearestValueToTime(14, kMaxDelta, &retrieved_item));
  EXPECT_EQ(retrieved_item.timestamp, 10);

  EXPECT_TRUE(!buffer_.getNearestValueToTime(16, kMaxDelta, &retrieved_item));
}

TEST_F(ThreadsafeTemporalBufferFixture, GetNearestValueToTimeMaxDeltaSmallWorks)
{
  addValue(TestData(0.05));
  addValue(TestData(0.06));
  addValue(TestData(0.057));

  const Timestamp kMaxDelta = 0.005;
  TestData retrieved_item;

  EXPECT_TRUE(buffer_.getNearestValueToTime(0.05, kMaxDelta, &retrieved_item));
  EXPECT_EQ(retrieved_item.timestamp, 0.05);

  EXPECT_FALSE(buffer_.getNearestValueToTime(0.04, kMaxDelta, &retrieved_item));  //!

  EXPECT_TRUE(buffer_.getNearestValueToTime(0.058, kMaxDelta, &retrieved_item));
  EXPECT_EQ(retrieved_item.timestamp, 0.057);

  EXPECT_FALSE(buffer_.getNearestValueToTime(0.066, kMaxDelta, &retrieved_item));  //!

  EXPECT_TRUE(buffer_.getNearestValueToTime(0.063, kMaxDelta, &retrieved_item));
  EXPECT_EQ(retrieved_item.timestamp, 0.06);

  Timestamp retrieved_timestamp;
  EXPECT_TRUE(buffer_.getNearestValueToTime(0.063, kMaxDelta, &retrieved_item, &retrieved_timestamp));
  EXPECT_EQ(retrieved_timestamp, 0.06);

  buffer_.clear();
}

TEST_F(ThreadsafeTemporalBufferFixture, GetValueAtOrBeforeTimeWorks)
{
  addValue(TestData(30));
  addValue(TestData(10));
  addValue(TestData(20));
  addValue(TestData(40));

  TestData retrieved_item;
  Timestamp timestamp;

  EXPECT_TRUE(buffer_.getValueAtOrBeforeTime(40, &timestamp, &retrieved_item));
  EXPECT_EQ(retrieved_item.timestamp, 40);
  EXPECT_EQ(timestamp, 40);

  EXPECT_TRUE(buffer_.getValueAtOrBeforeTime(50, &timestamp, &retrieved_item));
  EXPECT_EQ(retrieved_item.timestamp, 40);
  EXPECT_EQ(timestamp, 40);

  EXPECT_TRUE(buffer_.getValueAtOrBeforeTime(15, &timestamp, &retrieved_item));
  EXPECT_EQ(retrieved_item.timestamp, 10);
  EXPECT_EQ(timestamp, 10);

  EXPECT_TRUE(buffer_.getValueAtOrBeforeTime(10, &timestamp, &retrieved_item));
  EXPECT_EQ(retrieved_item.timestamp, 10);
  EXPECT_EQ(timestamp, 10);

  EXPECT_TRUE(!buffer_.getValueAtOrBeforeTime(5, &timestamp, &retrieved_item));
}

TEST_F(ThreadsafeTemporalBufferFixture, GetValueAtOrAfterTimeWorks)
{
  addValue(TestData(30));
  addValue(TestData(10));
  addValue(TestData(20));
  addValue(TestData(40));

  TestData retrieved_item;
  Timestamp timestamp;

  EXPECT_TRUE(buffer_.getValueAtOrAfterTime(10, &timestamp, &retrieved_item));
  EXPECT_EQ(retrieved_item.timestamp, 10);
  EXPECT_EQ(timestamp, 10);

  EXPECT_TRUE(buffer_.getValueAtOrAfterTime(5, &timestamp, &retrieved_item));
  EXPECT_EQ(retrieved_item.timestamp, 10);
  EXPECT_EQ(timestamp, 10);

  EXPECT_TRUE(buffer_.getValueAtOrAfterTime(35, &timestamp, &retrieved_item));
  EXPECT_EQ(retrieved_item.timestamp, 40);
  EXPECT_EQ(timestamp, 40);

  EXPECT_TRUE(buffer_.getValueAtOrAfterTime(40, &timestamp, &retrieved_item));
  EXPECT_EQ(retrieved_item.timestamp, 40);
  EXPECT_EQ(timestamp, 40);

  EXPECT_TRUE(!buffer_.getValueAtOrAfterTime(45, &timestamp, &retrieved_item));
}

TEST_F(ThreadsafeTemporalBufferFixture, GetOldestNewestValueWork)
{
  TestData retrieved_item;
  EXPECT_TRUE(!buffer_.getOldestValue(&retrieved_item));
  EXPECT_TRUE(!buffer_.getNewestValue(&retrieved_item));

  addValue(TestData(30.4));
  addValue(TestData(10.122));
  addValue(TestData(20.6));
  addValue(TestData(40.8));

  Timestamp timestamp = 0;
  EXPECT_TRUE(buffer_.getOldestValue(&retrieved_item, &timestamp));
  EXPECT_DOUBLE_EQ(retrieved_item.timestamp, 10.122);
  EXPECT_DOUBLE_EQ(timestamp, 10.122);

  EXPECT_TRUE(buffer_.getNewestValue(&retrieved_item, &timestamp));
  EXPECT_DOUBLE_EQ(timestamp, 40.8);
  EXPECT_DOUBLE_EQ(retrieved_item.timestamp, 40.8);
}

TEST_F(ThreadsafeTemporalBufferFixture, GetValuesBetweenTimesWorks)
{
  addValue(TestData(10));
  addValue(TestData(20));
  addValue(TestData(30));
  addValue(TestData(40));
  addValue(TestData(50));

  // Test aligned borders.
  /// When the user does not ask for the lower bound.
  /// Implicitly also checks that it is default behaviour.
  std::vector<TestData> values;
  EXPECT_TRUE(buffer_.getValuesBetweenTimes(10, 50, &values));
  EXPECT_EQ(values.size(), 3u);
  EXPECT_EQ(values[0].timestamp, 20);
  EXPECT_EQ(values[1].timestamp, 30);
  EXPECT_EQ(values[2].timestamp, 40);

  // Test aligned borders.
  /// When the user does ask for the lower bound.
  EXPECT_TRUE(buffer_.getValuesBetweenTimes(10, 50, &values, true));
  EXPECT_EQ(values.size(), 4u);
  EXPECT_EQ(values[0].timestamp, 10);
  EXPECT_EQ(values[1].timestamp, 20);
  EXPECT_EQ(values[2].timestamp, 30);
  EXPECT_EQ(values[3].timestamp, 40);

  // Test unaligned borders.
  /// When the user does not ask for the lower bound.
  /// Implicitly also checks that it is default behaviour.
  EXPECT_TRUE(buffer_.getValuesBetweenTimes(15, 45, &values));
  EXPECT_EQ(values.size(), 3u);
  EXPECT_EQ(values[0].timestamp, 20);
  EXPECT_EQ(values[1].timestamp, 30);
  EXPECT_EQ(values[2].timestamp, 40);

  // Test unaligned borders.
  /// When the user does ask for the lower bound.
  EXPECT_TRUE(buffer_.getValuesBetweenTimes(15, 45, &values));
  EXPECT_EQ(values.size(), 3u);
  EXPECT_EQ(values[0].timestamp, 20);
  EXPECT_EQ(values[1].timestamp, 30);
  EXPECT_EQ(values[2].timestamp, 40);

  // Test unsuccessful queries.
  // Lower border oob.
  EXPECT_TRUE(!buffer_.getValuesBetweenTimes(5, 45, &values));
  EXPECT_TRUE(!buffer_.getValuesBetweenTimes(5, 45, &values, true));
  // Higher border oob.
  EXPECT_TRUE(!buffer_.getValuesBetweenTimes(30, 55, &values));
  EXPECT_TRUE(!buffer_.getValuesBetweenTimes(30, 55, &values, true));
  EXPECT_TRUE(values.empty());

  // The method should check-fail when the buffer is empty.
  buffer_.clear();
  EXPECT_TRUE(!buffer_.getValuesBetweenTimes(10, 50, &values));
  EXPECT_TRUE(!buffer_.getValuesBetweenTimes(10, 50, &values, true));
  // EXPECT_DEATH(buffer_.getValuesBetweenTimes(40, 30, &values), "^");
}

TEST_F(ThreadsafeTemporalBufferFixture, MaintaingBufferLengthWorks)
{
  addValue(TestData(0));
  addValue(TestData(50));
  addValue(TestData(100));
  EXPECT_EQ(buffer_.size(), 3u);

  addValue(TestData(150));
  EXPECT_EQ(buffer_.size(), 3u);

  TestData retrieved_item;
  EXPECT_TRUE(buffer_.getOldestValue(&retrieved_item));
  EXPECT_EQ(retrieved_item.timestamp, 50);

  EXPECT_TRUE(buffer_.getNewestValue(&retrieved_item));
  EXPECT_EQ(retrieved_item.timestamp, 150);
}

TEST_F(ThreadsafeTemporalBufferFixture, DeltetingValuesAtTimestampsWork)
{
  addValue(TestData(12.4));
  addValue(TestData(0.001));
  addValue(TestData(56));
  addValue(TestData(21));

  TestData retrieved_item;
  Timestamp timestamp;

  EXPECT_TRUE(buffer_.getValueAtOrAfterTime(12.4, &timestamp, &retrieved_item));
  EXPECT_EQ(buffer_.size(), 4u);
  EXPECT_TRUE(buffer_.deleteValueAtTime(timestamp));
  EXPECT_TRUE(buffer_.getValueAtOrBeforeTime(12.4, &timestamp, &retrieved_item));
  EXPECT_EQ(retrieved_item.timestamp, 0.001);
  EXPECT_FALSE(buffer_.getNearestValueToTime(12.2, 1, &retrieved_item));
  EXPECT_EQ(buffer_.size(), 3u);

  // delete value using non-exact timestamp
  Timestamp stored_timestamp;
  EXPECT_TRUE(buffer_.getNearestValueToTime(59, 4, &retrieved_item, &stored_timestamp));
  EXPECT_EQ(retrieved_item.timestamp, 56);
  EXPECT_EQ(stored_timestamp, 56);
  EXPECT_TRUE(buffer_.deleteValueAtTime(stored_timestamp));
  EXPECT_EQ(buffer_.size(), 2u);
}

}  // namespace dyno
```

## File: vision_map/test_map.cc
```
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

#include <glog/logging.h>
#include <gtest/gtest.h>

#include <exception>

#include "dynosam/common/Map.hpp"
#include "dynosam/common/Types.hpp"
#include "dynosam/frontend/vision/Frame.hpp"
#include "internal/helpers.hpp"

using namespace dyno;

TEST(Map, basicAddOnlyStatic) {
  //   GenericTrackedStatusVector<VisualMeasurementStatus<Keypoint>>
  //   measurements;

  StatusKeypointVector measurements;

  TrackletIds expected_tracklets;
  // 10 measurements with unique tracklets at frame 0
  for (size_t i = 0; i < 10; i++) {
    measurements.push_back(
        dyno_testing::makeStatusKeypointMeasurement(i, background_label, 0));
    expected_tracklets.push_back(i);
  }

  Map2d::Ptr map = Map2d::create();

  map->updateObservations(measurements);

  EXPECT_TRUE(map->frameExists(0));
  EXPECT_FALSE(map->frameExists(1));

  EXPECT_TRUE(map->landmarkExists(0));
  EXPECT_TRUE(map->landmarkExists(9));
  EXPECT_FALSE(map->landmarkExists(10));

  EXPECT_EQ(map->getStaticTrackletsByFrame(0), expected_tracklets);

  // expected tracklets in frame 0
  TrackletIds expected_tracklets_f0 = expected_tracklets;

  TrackletIds expected_tracklets_f1;
  // add another 5 points at frame 1
  measurements.clear();
  for (size_t i = 0; i < 5; i++) {
    measurements.push_back(
        dyno_testing::makeStatusKeypointMeasurement(i, background_label, 1));

    expected_tracklets.push_back(i);
    expected_tracklets_f1.push_back(i);
  }

  // apply update
  map->updateObservations(measurements);

  EXPECT_EQ(map->getStaticTrackletsByFrame(0), expected_tracklets_f0);
  EXPECT_EQ(map->getStaticTrackletsByFrame(1), expected_tracklets_f1);

  // check for frames in some landmarks
  // should be seen in frames 0 and 1
  LandmarkNode<Keypoint>::Ptr lmk1 = map->getLandmark(0);
  std::vector<FrameId> lmk_1_seen_frames =
      lmk1->getSeenFrames().collectIds<FrameId>();
  std::vector<FrameId> lmk_1_seen_frames_expected = {0, 1};
  EXPECT_EQ(lmk_1_seen_frames, lmk_1_seen_frames_expected);

  // should be seen in frames 0
  LandmarkNode<Keypoint>::Ptr lmk6 = map->getLandmark(6);
  std::vector<FrameId> lmk_6_seen_frames =
      lmk6->getSeenFrames().collectIds<FrameId>();
  std::vector<FrameId> lmk_6_seen_frames_expected = {0};
  EXPECT_EQ(lmk_6_seen_frames, lmk_6_seen_frames_expected);

  // check that the frames here are the ones in the map
  EXPECT_EQ(map->getFrame(lmk_1_seen_frames.at(0)),
            map->getFrame(lmk_6_seen_frames.at(0)));

  // finally check that there are no objects
  EXPECT_EQ(map->getFrame(lmk_1_seen_frames.at(0))->objects_seen.size(), 0);
  EXPECT_EQ(map->numObjectsSeen(), 0u);
}

TEST(Map, setStaticOrdering) {
  // add frames out of order
  Map2d::Ptr map = Map2d::create();

  StatusKeypointVector measurements;
  // frame 0
  measurements.push_back(dyno_testing::makeStatusKeypointMeasurement(1, 0, 0));
  // frame 2
  measurements.push_back(dyno_testing::makeStatusKeypointMeasurement(1, 0, 2));
  // frame 1
  measurements.push_back(dyno_testing::makeStatusKeypointMeasurement(1, 0, 1));
  // frame 3
  measurements.push_back(dyno_testing::makeStatusKeypointMeasurement(1, 0, 3));
  map->updateObservations(measurements);

  LandmarkNode<Keypoint>::Ptr lmk = map->getLandmark(1);
  EXPECT_TRUE(lmk != nullptr);

  FrameIds expected_frame_ids = {0, 1, 2, 3};
  EXPECT_EQ(lmk->getSeenFrames().collectIds<FrameId>(), expected_frame_ids);

  EXPECT_EQ(lmk->getSeenFrames().getFirstIndex(), 0u);
  EXPECT_EQ(lmk->getSeenFrames().getLastIndex(), 3u);
}

TEST(Map, basicObjectAdd) {
  Map2d::Ptr map = Map2d::create();
  StatusKeypointVector measurements;
  // add two dynamic points on object 1 and frames 0 and 1
  // tracklet 0, object 1 frame 0
  measurements.push_back(dyno_testing::makeStatusKeypointMeasurement(0, 1, 0));
  // tracklet 0, object 1 frame 1
  measurements.push_back(dyno_testing::makeStatusKeypointMeasurement(0, 1, 1));

  map->updateObservations(measurements);
  EXPECT_EQ(map->numObjectsSeen(), 1u);
  EXPECT_TRUE(map->objectExists(1));

  ObjectNode2d::Ptr object1 = map->getObject(1);
  // object 1 should have 1 point seen at frames 0 and 1
  EXPECT_EQ(object1->dynamic_landmarks.collectIds<TrackletId>(),
            TrackletIds{0});
  EXPECT_EQ(object1->dynamic_landmarks.size(), 1u);

  // now check that the frames also have these measurements
  FrameNode2d::Ptr frame_0 = map->getFrame(0);
  FrameNode2d::Ptr frame_1 = map->getFrame(1);
  EXPECT_TRUE(frame_0 != nullptr);
  EXPECT_TRUE(frame_1 != nullptr);

  EXPECT_EQ(frame_0->dynamic_landmarks.collectIds<TrackletId>(),
            TrackletIds{0});
  EXPECT_EQ(frame_1->dynamic_landmarks.collectIds<TrackletId>(),
            TrackletIds{0});

  // sanity check that there are no static points
  EXPECT_EQ(frame_0->static_landmarks.size(), 0u);
  EXPECT_EQ(frame_1->static_landmarks.size(), 0u);

  // check object id and seen frames
  LandmarkNode2d::Ptr lmk_0 = map->getLandmark(0);
  EXPECT_EQ(lmk_0->object_id, 1);  // object Id 1;
  EXPECT_EQ(lmk_0->getSeenFrames().collectIds<FrameId>(),
            FrameIds({0, 1}));  // seen frames

  // finally check that the landmark referred to by the frames are the same one
  // as getLandmark(0) this also implicitly tests FastMapNodeSet::find(index)
  auto frame_0_dynamic_lmks = frame_0->dynamic_landmarks;
  auto frame_1_dynamic_lmks = frame_1->dynamic_landmarks;

  // look from the lmk with id 0
  auto lmk_itr_frame_0 = frame_0_dynamic_lmks.find(0);
  auto lmk_itr_frame_1 = frame_1_dynamic_lmks.find(0);
  // should not be at the end as we have this landmark
  EXPECT_FALSE(lmk_itr_frame_0 == frame_0_dynamic_lmks.end());
  EXPECT_FALSE(lmk_itr_frame_1 == frame_1_dynamic_lmks.end());

  // check the lmk is the one we got from the map
  EXPECT_EQ(lmk_0, *lmk_itr_frame_0);
  EXPECT_EQ(lmk_0, *lmk_itr_frame_1);
}

TEST(Map, framesSeenDuplicates) {
  Map2d::Ptr map = Map2d::create();
  LandmarkNode2d::Ptr landmark_node =
      std::make_shared<LandmarkNode2d>(map->getptr());
  landmark_node->tracklet_id = 0;
  landmark_node->object_id = 0;

  EXPECT_EQ(landmark_node->numObservations(), 0);

  FrameNode2d::Ptr frame_node = std::make_shared<FrameNode2d>(map->getptr());
  frame_node->frame_id = 0;

  landmark_node->add(frame_node, Keypoint());

  EXPECT_EQ(landmark_node->numObservations(), 1);
  EXPECT_EQ(*landmark_node->getSeenFrames().begin(), frame_node);
  EXPECT_EQ(landmark_node->getMeasurements().size(), 1);

  // now add the same frame again
  EXPECT_THROW({ landmark_node->add(frame_node, Keypoint()); },
               DynosamException);
}

TEST(Map, objectSeenFrames) {
  Map2d::Ptr map = Map2d::create();
  StatusKeypointVector measurements;

  // add 2 objects
  // object 1 seen at frames 0 and 1
  // tracklet 0, object 1 frame 0
  measurements.push_back(dyno_testing::makeStatusKeypointMeasurement(0, 1, 0));
  // tracklet 0, object 1 frame 1
  measurements.push_back(dyno_testing::makeStatusKeypointMeasurement(0, 1, 1));
  // tracklet 0 has two observations

  // object 2 seen at frames 1 and 2
  // tracklet 1, object 2 frame 1
  // tracklet 1 has 1 observations
  measurements.push_back(dyno_testing::makeStatusKeypointMeasurement(1, 2, 1));
  // tracklet 2, object 2 frame 2
  // tracklet 2 has 1 observations
  measurements.push_back(dyno_testing::makeStatusKeypointMeasurement(2, 2, 2));

  // object 3 seen at frames 0, 1, 2
  // tracklet 3, object 3 frame 0
  measurements.push_back(dyno_testing::makeStatusKeypointMeasurement(3, 3, 0));
  // tracklet 3, object 3 frame 1
  measurements.push_back(dyno_testing::makeStatusKeypointMeasurement(3, 3, 1));
  // tracklet 3, object 3 frame 2
  measurements.push_back(dyno_testing::makeStatusKeypointMeasurement(3, 3, 2));
  // tracklet 3 has 3 observations

  map->updateObservations(measurements);
  EXPECT_EQ(map->numObjectsSeen(), 3u);

  ObjectNode2d::Ptr object1 = map->getObject(1);
  ObjectNode2d::Ptr object2 = map->getObject(2);
  ObjectNode2d::Ptr object3 = map->getObject(3);

  FrameNodePtrSet<Keypoint> expected_frame_set_object1;
  expected_frame_set_object1.insert(CHECK_NOTNULL(map->getFrame(0)));
  expected_frame_set_object1.insert(CHECK_NOTNULL(map->getFrame(1)));

  FrameNodePtrSet<Keypoint> expected_frame_set_object2;
  expected_frame_set_object2.insert(CHECK_NOTNULL(map->getFrame(1)));
  expected_frame_set_object2.insert(CHECK_NOTNULL(map->getFrame(2)));

  FrameNodePtrSet<Keypoint> expected_frame_set_object3;
  expected_frame_set_object3.insert(CHECK_NOTNULL(map->getFrame(0)));
  expected_frame_set_object3.insert(CHECK_NOTNULL(map->getFrame(1)));
  expected_frame_set_object3.insert(CHECK_NOTNULL(map->getFrame(2)));

  EXPECT_EQ(object1->getSeenFrames(), expected_frame_set_object1);
  EXPECT_EQ(object2->getSeenFrames(), expected_frame_set_object2);
  EXPECT_EQ(object3->getSeenFrames(), expected_frame_set_object3);

  // frame 0 has seen object 1 and 3 -the reverse of the above testt
  EXPECT_EQ(map->getFrame(0)->objects_seen,
            ObjectNodePtrSet<Keypoint>({object1, object3}));
  // frame 1 has seen object 1, 2 and 3
  EXPECT_EQ(map->getFrame(1)->objects_seen,
            ObjectNodePtrSet<Keypoint>({object1, object3, object2}));
  // frame 2 has seen object 2 and 3
  EXPECT_EQ(map->getFrame(2)->objects_seen,
            ObjectNodePtrSet<Keypoint>({object2, object3}));

  // check object observation functions
  auto frame0 = map->getFrame(0);
  auto frame1 = map->getFrame(1);
  auto frame2 = map->getFrame(2);
  // check object observed frame 0
  EXPECT_TRUE(frame0->objectObserved(1));
  EXPECT_TRUE(frame0->objectObserved(3));
  EXPECT_FALSE(frame0->objectObserved(2));

  // check object observed frame 1 (all)
  EXPECT_TRUE(frame1->objectObserved(1));
  EXPECT_TRUE(frame1->objectObserved(3));
  EXPECT_TRUE(frame1->objectObserved(2));
  // check object observed frame 2
  EXPECT_TRUE(frame2->objectObserved(2));
  EXPECT_TRUE(frame2->objectObserved(3));
  EXPECT_FALSE(frame2->objectObserved(1));

  // check observed in previous (in frame 0, there is no previous so all
  // false!!)
  EXPECT_FALSE(frame0->objectObservedInPrevious(1));
  EXPECT_FALSE(frame0->objectObservedInPrevious(3));

  // both object1 and 3 appear in frame 0 but not object 2
  EXPECT_TRUE(frame1->objectObservedInPrevious(1));
  EXPECT_TRUE(frame1->objectObservedInPrevious(3));
  EXPECT_FALSE(frame1->objectObservedInPrevious(2));

  // all objects are observed at frame 1
  EXPECT_TRUE(frame2->objectObservedInPrevious(1));
  EXPECT_TRUE(frame2->objectObservedInPrevious(3));
  EXPECT_TRUE(frame2->objectObservedInPrevious(2));

  // check objectMotionExpected (i.e objects are observed at both frames)
  EXPECT_FALSE(frame0->objectMotionExpected(1));
  EXPECT_FALSE(frame0->objectMotionExpected(3));

  // object 1 and 3 seen at frames 0 and 1, but not object 2
  EXPECT_TRUE(frame1->objectMotionExpected(1));
  EXPECT_TRUE(frame1->objectMotionExpected(3));
  EXPECT_FALSE(frame1->objectMotionExpected(2));
  // object 2 and 3 seen at frames 1 and 2, but not object 1
  EXPECT_FALSE(frame2->objectMotionExpected(1));
  EXPECT_TRUE(frame2->objectMotionExpected(3));
  EXPECT_TRUE(frame2->objectMotionExpected(2));
}

TEST(Map, getLandmarksSeenAtFrame) {
  Map2d::Ptr map = Map2d::create();
  StatusKeypointVector measurements;

  // add 2 objects
  // object 1 seen at frames 0 and 1
  // tracklet 0, object 1 frame 0
  measurements.push_back(dyno_testing::makeStatusKeypointMeasurement(0, 1, 0));
  // tracklet 0, object 1 frame 1
  measurements.push_back(dyno_testing::makeStatusKeypointMeasurement(0, 1, 1));

  // object 2 seen at frames 1 and 2
  // tracklet 1, object 2 frame 1
  measurements.push_back(dyno_testing::makeStatusKeypointMeasurement(1, 2, 1));
  // tracklet 2, object 2 frame 1
  measurements.push_back(dyno_testing::makeStatusKeypointMeasurement(2, 2, 1));

  // object 3 seen at frames 0, 1, 2
  // tracklet 3, object 3 frame 0
  measurements.push_back(dyno_testing::makeStatusKeypointMeasurement(3, 3, 0));
  // tracklet 3, object 3 frame 1
  measurements.push_back(dyno_testing::makeStatusKeypointMeasurement(3, 3, 1));
  // tracklet 3, object 3 frame 1
  measurements.push_back(dyno_testing::makeStatusKeypointMeasurement(4, 3, 1));
  // tracklet 3 has 3 observations

  map->updateObservations(measurements);
  EXPECT_EQ(map->numObjectsSeen(), 3u);

  ObjectNode2d::Ptr object1 = map->getObject(1);
  ObjectNode2d::Ptr object2 = map->getObject(2);
  ObjectNode2d::Ptr object3 = map->getObject(3);

  LandmarkNodePtrSet<Keypoint> expected_lmk_set_object1_frame0;
  expected_lmk_set_object1_frame0.insert(CHECK_NOTNULL(map->getLandmark(0)));

  LandmarkNodePtrSet<Keypoint> expected_lmk_set_object1_frame1;
  expected_lmk_set_object1_frame1.insert(CHECK_NOTNULL(map->getLandmark(0)));

  LandmarkNodePtrSet<Keypoint> expected_lmk_set_object2_frame1;
  expected_lmk_set_object2_frame1.insert(CHECK_NOTNULL(map->getLandmark(1)));
  expected_lmk_set_object2_frame1.insert(CHECK_NOTNULL(map->getLandmark(2)));

  LandmarkNodePtrSet<Keypoint> expected_lmk_set_object3_frame0;
  expected_lmk_set_object3_frame0.insert(CHECK_NOTNULL(map->getLandmark(3)));

  LandmarkNodePtrSet<Keypoint> expected_lmk_set_object3_frame1;
  expected_lmk_set_object3_frame1.insert(CHECK_NOTNULL(map->getLandmark(3)));
  expected_lmk_set_object3_frame1.insert(CHECK_NOTNULL(map->getLandmark(4)));

  EXPECT_EQ(object1->getLandmarksSeenAtFrame(0),
            expected_lmk_set_object1_frame0);
  EXPECT_EQ(object1->getLandmarksSeenAtFrame(1),
            expected_lmk_set_object1_frame1);
  EXPECT_EQ(object1->getLandmarksSeenAtFrame(2),
            LandmarkNodePtrSet<Keypoint>{});
  EXPECT_EQ(object2->getLandmarksSeenAtFrame(1),
            expected_lmk_set_object2_frame1);
  EXPECT_EQ(object3->getLandmarksSeenAtFrame(0),
            expected_lmk_set_object3_frame0);
  EXPECT_EQ(object3->getLandmarksSeenAtFrame(1),
            expected_lmk_set_object3_frame1);
}

// TODO: bring back!!
//  TEST(Map, testSimpleEstimateAccessWithPose) {
//      Map2d::Ptr map = Map2d::create();
//      StatusKeypointMeasurements measurements;

//     //TODO:frame cannot be 0? what is invalid frame then?
//     EXPECT_EQ(map->lastEstimateUpdate(), 0u);

//     //tracklet 0, static, frame
//     measurements.push_back(dyno_testing::makeStatusKeypointMeasurement(0, 0,
//     0)); map->updateObservations(measurements);

//     auto frame_0 = map->getFrame(0);
//     auto pose_0_query = frame_0->getPoseEstimate();
//     EXPECT_FALSE(pose_0_query);
//     EXPECT_FALSE(pose_0_query.isValid());

//     gtsam::Values estimate;

//     //some random pose
//     gtsam::Rot3 R = gtsam::Rot3::Rodrigues(0.3,0.4,-0.5);
//     gtsam::Point3 t(3.5,-8.2,4.2);
//     gtsam::Pose3 pose_0_actual(R,t);
//     gtsam::Key pose_0_key = CameraPoseSymbol(0);
//     estimate.insert(pose_0_key, pose_0_actual);

//     map->updateEstimates(estimate, gtsam::NonlinearFactorGraph{}, 0);
//     pose_0_query = frame_0->getPoseEstimate();

//     EXPECT_TRUE(pose_0_query);
//     EXPECT_TRUE(pose_0_query.isValid());
//     EXPECT_TRUE(gtsam::assert_equal(pose_0_query.get(), pose_0_actual));
//     EXPECT_EQ(pose_0_query.key_, pose_0_key);

// }

// TEST(Map, testEstimateWithStaticAndDynamicViaLmk) {
//     Map2d::Ptr map = Map2d::create();
//     StatusKeypointMeasurements measurements;

//     //tracklet 0, static, frame 0
//     measurements.push_back(dyno_testing::makeStatusKeypointMeasurement(0, 0,
//     0));
//     //tracklet 0, static, frame 1
//     measurements.push_back(dyno_testing::makeStatusKeypointMeasurement(0, 0,
//     1));
//     //static points should return the same estimate

//     //tracklet 1, dynamic, frame 0
//     measurements.push_back(dyno_testing::makeStatusKeypointMeasurement(1, 1,
//     0));
//     //tracklet 1, dynamic, frame 1
//     measurements.push_back(dyno_testing::makeStatusKeypointMeasurement(1, 1,
//     1));
//     //dynamic points should return different estimates

//     gtsam::Point3 static_lmk(0, 0, 0);
//     gtsam::Point3 dynamic_lmk_1(0, 0, 1);
//     gtsam::Point3 dynamic_lmk_2(0, 0, 2);

//     gtsam::Values estimate;
//     estimate.insert(StaticLandmarkSymbol(0), static_lmk);
//     estimate.insert(DynamicLandmarkSymbol(0, 1), dynamic_lmk_1);
//     estimate.insert(DynamicLandmarkSymbol(1, 1), dynamic_lmk_2);

//     map->updateObservations(measurements);
//     map->updateEstimates(estimate,gtsam::NonlinearFactorGraph{}, 0 );

//     LandmarkNode2d::Ptr static_lmk_node = map->getLandmark(0);
//     EXPECT_TRUE(static_lmk_node->isStatic());
//     auto estimate_query = static_lmk_node->getStaticLandmarkEstimate();
//     EXPECT_TRUE(estimate_query);
//     EXPECT_TRUE(estimate_query.isValid());
//     EXPECT_TRUE(gtsam::assert_equal(estimate_query.get(), static_lmk));

//     LandmarkNode2d::Ptr dynamic_lmk_1_node = map->getLandmark(1);
//     EXPECT_FALSE(dynamic_lmk_1_node->isStatic());
//     estimate_query = dynamic_lmk_1_node->getDynamicLandmarkEstimate(0);
//     EXPECT_TRUE(estimate_query);
//     EXPECT_TRUE(estimate_query.isValid());
//     EXPECT_TRUE(gtsam::assert_equal(estimate_query.get(), dynamic_lmk_1));

//     LandmarkNode2d::Ptr dynamic_lmk_2_node = map->getLandmark(1);
//     EXPECT_FALSE(dynamic_lmk_2_node->isStatic());
//     estimate_query = dynamic_lmk_2_node->getDynamicLandmarkEstimate(1);
//     EXPECT_TRUE(estimate_query);
//     EXPECT_TRUE(estimate_query.isValid());
//     EXPECT_TRUE(gtsam::assert_equal(estimate_query.get(), dynamic_lmk_2));

// }

// TEST(Map, testEstimateWithStaticAndDynamicViaFrame) {

// }

// TEST(Map, testObjectNodeWithOnlyMotion) {
//     Map2d::Ptr map = Map2d::create();

//     //create dynamic observations for object 1 at frames 0 and 1
//     StatusKeypointMeasurements measurements;
//      //tracklet 1, dynamic, frame 0
//     measurements.push_back(dyno_testing::makeStatusKeypointMeasurement(1, 1,
//     0));
//     //tracklet 1, dynamic, frame 1
//     measurements.push_back(dyno_testing::makeStatusKeypointMeasurement(1, 1,
//     1));

//     gtsam::Values estimate;
//     //add motion only at frame 1
//     const gtsam::Pose3 pose_estimate =
//     utils::createRandomAroundIdentity<gtsam::Pose3>(0.3);
//     estimate.insert(ObjectMotionSymbol(1, 1), pose_estimate);

//     map->updateObservations(measurements);
//     map->updateEstimates(estimate,gtsam::NonlinearFactorGraph{}, 1 );

//     ObjectNode2d::Ptr object1 = map->getObject(1);
//     EXPECT_TRUE(object1->getMotionEstimate(1));

//     gtsam::Pose3 recovered_pose_estimate;
//     EXPECT_TRUE(object1->hasMotionEstimate(1, &recovered_pose_estimate));
//     //check nullptr version
//     EXPECT_TRUE(object1->hasMotionEstimate(1));
//     EXPECT_TRUE(gtsam::assert_equal(recovered_pose_estimate, pose_estimate));

//     //should not throw exception as we have seen this obejct at frames 0 and
//     1 (based on the measurements)
//     //we just dont have an estimate of the pose
//     EXPECT_FALSE(object1->getPoseEstimate(0));
//     EXPECT_FALSE(object1->getPoseEstimate(1));
// }

// TEST(Map, testObjectNodeWithOnlyPose) {
//     Map2d::Ptr map = Map2d::create();

//     //create dynamic observations for object 1 at frames 0 and 1
//     StatusKeypointMeasurements measurements;
//      //tracklet 1, dynamic, frame 0
//     measurements.push_back(dyno_testing::makeStatusKeypointMeasurement(1, 1,
//     0));
//     //tracklet 1, dynamic, frame 1
//     measurements.push_back(dyno_testing::makeStatusKeypointMeasurement(1, 1,
//     1));

//     gtsam::Values estimate;
//     //add motion only at frame 1
//     const gtsam::Pose3 pose_estimate =
//     utils::createRandomAroundIdentity<gtsam::Pose3>(0.3);
//     estimate.insert(ObjectPoseSymbol(1, 1), pose_estimate);

//     map->updateObservations(measurements);
//     map->updateEstimates(estimate,gtsam::NonlinearFactorGraph{}, 1 );

//     ObjectNode2d::Ptr object1 = map->getObject(1);
//     EXPECT_TRUE(object1->getPoseEstimate(1));

//     gtsam::Pose3 recovered_pose_estimate;
//     EXPECT_TRUE(object1->hasPoseEstimate(1, &recovered_pose_estimate));
//     //check nullptr version
//     EXPECT_TRUE(object1->hasPoseEstimate(1));
//     EXPECT_TRUE(gtsam::assert_equal(recovered_pose_estimate, pose_estimate));

//     //should not throw exception as we have seen this obejct at frames 0 and
//     1 (based on the measurements)
//     //we just dont have an estimate of the motion
//     EXPECT_FALSE(object1->getMotionEstimate(1));
// }
```

## File: vision_map/test_tools.cc
```
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
#include <gtest/gtest.h>
#include <gtsam/geometry/StereoCamera.h>

#include <cmath>

#include "dynosam/common/Types.hpp"
#include "dynosam/frontend/vision/VisionTools.hpp"
#include "internal/helpers.hpp"

using namespace dyno;

TEST(VisionTools, determineOutlierIdsBasic) {
  TrackletIds tracklets = {1, 2, 3, 4, 5};
  TrackletIds inliers = {1, 2};

  TrackletIds expected_outliers = {3, 4, 5};
  TrackletIds outliers;
  determineOutlierIds(inliers, tracklets, outliers);
  EXPECT_EQ(expected_outliers, outliers);
}

TEST(VisionTools, determineOutlierIdsUnorderd) {
  TrackletIds tracklets = {12, 45, 1, 85, 3, 100};
  TrackletIds inliers = {3, 1, 100};

  TrackletIds expected_outliers = {12, 45, 85};
  TrackletIds outliers;
  determineOutlierIds(inliers, tracklets, outliers);
  EXPECT_EQ(expected_outliers, outliers);
}

TEST(VisionTools, determineOutlierIdsNoSubset) {
  TrackletIds tracklets = {12, 45, 1, 85, 3, 100};
  TrackletIds inliers = {12, 45, 1, 85, 3, 100};

  TrackletIds outliers = {4, 5, 6};  // also add a test that outliers is cleared
  determineOutlierIds(inliers, tracklets, outliers);
  EXPECT_TRUE(outliers.empty());
}

TEST(VisionTools, testMacVOUncertaintyPropogation) {
  Camera camera = dyno_testing::makeDefaultCamera();
  CameraParams params = camera.getParams();
  auto camera_impl = camera.getImplCamera();

  // make stereo camera
  const double base_line = 0.5;
  gtsam::Cal3_S2Stereo::shared_ptr stereo_params =
      boost::make_shared<gtsam::Cal3_S2Stereo>(
          params.fx(), params.fy(), 0, params.cu(), params.cv(), base_line);
  gtsam::StereoCamera stereo_camera(gtsam::Pose3::Identity(), stereo_params);

  Feature feature;
  Keypoint kp(params.cu(), params.cv() + 20);
  feature.keypoint(kp);
  feature.depth(1.0);

  // sigmas squared
  double kp_sigma_2 = 0.1;
  double depth_sigma_2 = 0.000005;

  // first check diagonal components of proposed macv matrix
  gtsam::Matrix32 J_keypoint;
  gtsam::Matrix31 J_depth;
  gtsam::Point3 landmark =
      camera_impl->backproject(feature.keypoint(), feature.depth(), boost::none,
                               J_keypoint, J_depth, boost::none);

  gtsam::StereoPoint2 stereo_kp = stereo_camera.project(landmark);
  EXPECT_EQ(kp(0), stereo_kp.uL());
  EXPECT_EQ(kp(1), stereo_kp.v());

  gtsam::Matrix33 J_stereo_point;
  stereo_camera.backproject2(stereo_kp, boost::none, J_stereo_point);

  gtsam::Point3 calc_landmark(
      ((feature.keypoint()(0) - params.cu()) * feature.depth()) / params.fx(),
      ((feature.keypoint()(1) - params.cv()) * feature.depth()) / params.fy(),
      feature.depth());
  EXPECT_TRUE(gtsam::assert_equal(calc_landmark, landmark));

  // form measurement covariance matrices
  gtsam::Matrix22 pixel_covariance_matrix;
  pixel_covariance_matrix << kp_sigma_2, 0.0, 0.0, kp_sigma_2;

  gtsam::Matrix33 stereo_pixel_covariance_matrix;
  stereo_pixel_covariance_matrix << kp_sigma_2, 0.0, 0.0, 0, kp_sigma_2, 0, 0,
      0, kp_sigma_2;

  // for depth uncertainty, we model it as a quadratic increase with distnace
  // double depth_covariance = depth_sigma * std::pow(depth, 2);
  double depth_covariance = depth_sigma_2;
  LOG(INFO) << "J_keypoint " << J_keypoint;
  // calcualte 3x3 covairance matrix
  gtsam::Matrix33 covariance =
      J_keypoint * pixel_covariance_matrix * J_keypoint.transpose();
  // J_depth * depth_covariance * J_depth.transpose();

  gtsam::Matrix33 stereo_covariance = J_stereo_point *
                                      stereo_pixel_covariance_matrix *
                                      J_stereo_point.transpose();

  LOG(INFO) << "Jacobian cov " << covariance;
  LOG(INFO) << "Stereo Jacobian cov " << stereo_covariance;

  double d_2 = std::pow(feature.depth(), 2);
  double u_2 = std::pow(feature.keypoint()(0), 2);
  double v_2 = std::pow(feature.keypoint()(1), 2);
  double fx_2 = std::pow(params.fx(), 2);
  double fy_2 = std::pow(params.fy(), 2);
  double cx_2 = std::pow(params.cu(), 2);
  double cy_2 = std::pow(params.cv(), 2);

  double mac_v_sigma_x = ((kp_sigma_2 + d_2) * (depth_sigma_2 + u_2) -
                          u_2 * d_2 + cx_2 * depth_sigma_2) /
                         fx_2;
  double mac_v_sigma_y = ((kp_sigma_2 + d_2) * (depth_sigma_2 + v_2) -
                          v_2 * d_2 + cy_2 * depth_sigma_2) /
                         fy_2;
  double mac_v_sigma_z = depth_sigma_2;

  gtsam::Matrix33 macvo_covariance;
  macvo_covariance << mac_v_sigma_x, 0, 0, 0, mac_v_sigma_y, 0, 0, 0,
      mac_v_sigma_z;

  LOG(INFO) << "macvo cov " << macvo_covariance;
}
```

## File: visualization/test_viz.cc
```
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

#include <glog/logging.h>
#include <gtest/gtest.h>

#include "dynosam/visualizer/ColourMap.hpp"

using namespace dyno;

TEST(Color, testRGBDDownCasting) {
  RGBA<uint8_t> rgbd_int(255, 255, 0);
  EXPECT_EQ(rgbd_int.r, 255);
  EXPECT_EQ(rgbd_int.g, 255);
  EXPECT_EQ(rgbd_int.b, 0);
  EXPECT_EQ(rgbd_int.a, 255);

  RGBA<float> rgbd_float(rgbd_int);
  EXPECT_EQ(rgbd_float.r, 1.0);
  EXPECT_EQ(rgbd_float.g, 1.0);
  EXPECT_EQ(rgbd_float.b, 0);
  EXPECT_EQ(rgbd_float.a, 1.0);
}

TEST(Color, testRGBDDownCasting1) {
  RGBA<uint8_t> rgbd_int(128, 100, 10);
  EXPECT_EQ(rgbd_int.r, 128);
  EXPECT_EQ(rgbd_int.g, 100);
  EXPECT_EQ(rgbd_int.b, 10);
  EXPECT_EQ(rgbd_int.a, 255);

  RGBA<float> rgbd_float(rgbd_int);
  EXPECT_FLOAT_EQ(rgbd_float.r, 128.0 / 255.0);
  EXPECT_FLOAT_EQ(rgbd_float.g, 100.0 / 255.0);
  EXPECT_FLOAT_EQ(rgbd_float.b, 10.0 / 255.0);
  EXPECT_FLOAT_EQ(rgbd_float.a, 1.0);
}

TEST(Color, testRGBDUpCasting) {
  RGBA<float> rgbd_float(0.5, 0.2, 0.1);
  EXPECT_FLOAT_EQ(rgbd_float.r, 0.5);
  EXPECT_FLOAT_EQ(rgbd_float.g, 0.2);
  EXPECT_FLOAT_EQ(rgbd_float.b, 0.1);
  EXPECT_FLOAT_EQ(rgbd_float.a, 1.0);

  RGBA<uint8_t> rgbd_int(rgbd_float);
  EXPECT_EQ(rgbd_int.r, static_cast<uint8_t>(0.5 * 255.0));
  EXPECT_EQ(rgbd_int.g, static_cast<uint8_t>(0.2 * 255.0));
  EXPECT_EQ(rgbd_int.b, static_cast<uint8_t>(0.1 * 255.0));
  EXPECT_EQ(rgbd_int.a, static_cast<uint8_t>(255.0));
}
```

## File: test_main.cc
```
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

#include "internal/helpers.hpp"

#include <gtest/gtest.h>
#include <glog/logging.h>

#include "rclcpp/rclcpp.hpp"


DEFINE_string(test_data_path, getTestDataPath(), "Path to data for unit tests.");

int main(int argc, char** argv)
{
  // rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  google::InitGoogleLogging(argv[0]);

  FLAGS_logtostderr = 1;
  FLAGS_colorlogtostderr = 1;
  FLAGS_log_prefix = 1;
  FLAGS_v = 1;
  google::ParseCommandLineFlags(&argc, &argv, true);
  return RUN_ALL_TESTS();
}
```
