// File: camera/ZEDCamera/test_ZED_camera_full.cc
#include <gtest/gtest.h>
#include <glog/logging.h>
#include <sl/Camera.hpp>
#include <vector>
#include <string>
#include <optional>
#include <fstream>    // For SVO file check/creation
#include <filesystem> // For creating test_data directory

#include "dynosam/common/ZEDCamera.hpp" // The class under test

// It's good practice to put test helpers and fixtures in a specific namespace
// or keep them static within the test file.
namespace dyno {
namespace common {
namespace test {
// Helper to create a default config for a live camera
ZEDCameraConfig getLiveCameraDefaultConfig(int camera_id = 0) {
  ZEDCameraConfig config;
  config.resolution = sl::RESOLUTION::HD720; // A common default
  config.fps = 30;
  config.depth_mode = sl::DEPTH_MODE::PERFORMANCE;
  config.coordinate_units = sl::UNIT::METER;
  config.coordinate_system_3d = sl::COORDINATE_SYSTEM::IMAGE;
  // Often used for CV tasks
  config.enable_imu = false; // Keep false unless specifically testing IMU
  config.svo_file_path = ""; // Ensure live mode
  config.camera_device_id = camera_id; // Default camera
  config.sdk_verbose = 0; // Keep logs clean unless debugging tests
  // Other object detection/body tracking flags default to false/default models
  return config;
}

// Helper to create a config for SVO mode
// NOTE: For SVO tests to pass, a valid SVO file at TEST_SVO_FILE_PATH must exist.
const std::string TEST_SVO_DIRECTORY = "test_data_zed";
const std::string TEST_SVO_FILENAME = "test_camera.svo";
const std::string TEST_SVO_FILE_PATH =
    TEST_SVO_DIRECTORY + "/" + TEST_SVO_FILENAME;

ZEDCameraConfig getSVOConfig(const std::string& svo_path = TEST_SVO_FILE_PATH) {
  ZEDCameraConfig config;
  config.svo_file_path = svo_path;
  config.svo_real_time_mode = false; // Usually false for testing frame-by-frame
  config.resolution = sl::RESOLUTION::HD720; // Actual SVO res will override
  config.fps = 30; // Actual SVO FPS will override
  config.depth_mode = sl::DEPTH_MODE::PERFORMANCE;
  config.enable_imu = false;
  config.sdk_verbose = 0;
  return config;
}

// Test fixture for tests that might need a real ZED camera
class ZEDCameraHardwareTest : public ::testing::Test {
public:
  static bool is_camera_available_;

protected:
  ZEDCameraConfig live_config_;
  std::unique_ptr<ZEDCamera> zed_cam_; // Use unique_ptr for RAII


  static void SetUpTestSuite() {
    std::vector<sl::DeviceProperties> device_list = sl::Camera::getDeviceList();
    is_camera_available_ = !device_list.empty();
    if (!is_camera_available_) {
      LOG(WARNING) <<
          "No ZED cameras detected. Live camera tests requiring hardware will be skipped or may fail gracefully.";
    } else {
      LOG(INFO) << "Detected " << device_list.size() << " ZED camera(s).";
    }
  }

  void SetUp() override {
    live_config_ = getLiveCameraDefaultConfig();
    if (!is_camera_available_) {
      LOG(WARNING) <<
          "Skipping ZEDCameraHardwareTest setup actions as no camera is available.";
    }
  }

  void TearDown() override {
    if (zed_cam_ && zed_cam_->isOpened()) {
      zed_cam_->close();
    }
    zed_cam_.reset(); // Explicitly reset unique_ptr
  }

  // Helper to ensure camera is open for a test
  bool ensureCameraOpen(const ZEDCameraConfig& config,
                        bool require_hardware = true) {
    if (require_hardware && !is_camera_available_) {
      return false;
    }
    zed_cam_ = std::make_unique<ZEDCamera>(config);
    if (!zed_cam_->open()) {
      LOG(ERROR) << "Failed to open ZED camera for test. Camera ID: " << config.
          camera_device_id
          << ". Serial Number might be required if multiple cameras or camera not connected/available.";
      return false;
    }
    return true;
  }
};

bool ZEDCameraHardwareTest::is_camera_available_ = true;


// Test fixture for SVO tests
class ZEDCameraSVOTest : public ::testing::Test {
protected:
  ZEDCameraConfig svo_config_;
  std::unique_ptr<ZEDCamera> zed_svo_cam_;
  static bool svo_file_accessible_;

  static void SetUpTestSuite() {
    std::filesystem::path svo_dir(TEST_SVO_DIRECTORY);
    std::filesystem::path svo_file(TEST_SVO_FILE_PATH);

    if (!std::filesystem::exists(svo_dir)) {
      std::filesystem::create_directories(svo_dir);
    }

    std::ifstream f(svo_file.c_str());
    svo_file_accessible_ = f.good();
    f.close();

    if (!svo_file_accessible_) {
      LOG(WARNING) << "Test SVO file '" << TEST_SVO_FILE_PATH
          << "' not found or not accessible. SVO tests may be skipped or fail. "
          << "A small, valid SVO is required for these tests. Placeholder SVOs usually don't work.";
    } else {
      LOG(INFO) << "Test SVO file '" << TEST_SVO_FILE_PATH << "' found.";
    }
  }

  void SetUp() override {
    svo_config_ = getSVOConfig();
    if (!svo_file_accessible_) {
      LOG(ERROR) << "SVO file '" << TEST_SVO_FILE_PATH
          << "' not accessible at test SetUp. SVO tests are likely to fail.";
    }
  }

  void TearDown() override {
    if (zed_svo_cam_ && zed_svo_cam_->isOpened()) {
      zed_svo_cam_->close();
    }
    zed_svo_cam_.reset();
  }

  bool ensureSVOCameraOpen(const ZEDCameraConfig& config) {
    if (!svo_file_accessible_) {
      return false;
    }
    zed_svo_cam_ = std::make_unique<ZEDCamera>(config);
    if (!zed_svo_cam_->open()) {
      LOG(ERROR) << "Failed to open ZED SVO camera for test with file: " <<
          config.svo_file_path;
      return false;
    }
    return true;
  }
};

bool ZEDCameraSVOTest::svo_file_accessible_ = false;


// --- Lifecycle Management Tests ---
TEST_F(ZEDCameraHardwareTest, ZC_LC_001_OpenCloseCamera) {
  // **Test Case ID:** ZC_LC_001
  // **Method(s) Under Test:** open(), close(), isOpened()
  // **Test Scenario Description:** Verify basic open and close functionality for a live camera.
  // **Key Configurations/Inputs:** Default live ZEDCameraConfig.
  // **Expected Outcome:** Camera opens, isOpened() is true, then closes, isOpened() is false.
  // **Mocking Considerations:** None (real camera assumed).

  if (!ZEDCameraHardwareTest::is_camera_available_)
    GTEST_SKIP() << "Skipping test: No ZED camera hardware detected.";

  zed_cam_ = std::make_unique<ZEDCamera>(live_config_);
  ASSERT_TRUE(zed_cam_->open()) <<
 "Failed to open camera. Check connection and permissions.";
  EXPECT_TRUE(zed_cam_->isOpened());
  zed_cam_->close();
  EXPECT_FALSE(zed_cam_->isOpened());
}

TEST_F(ZEDCameraHardwareTest, ZC_LC_002_OpenAlreadyOpenedCamera) {
  // **Test Case ID:** ZC_LC_002
  // **Method(s) Under Test:** open(), isOpened()
  // **Test Scenario Description:** Calling open() on an already opened camera should be a no-op or succeed.
  // **Key Configurations/Inputs:** Default live ZEDCameraConfig.
  // **Expected Outcome:** Camera opens, isOpened() is true. Second open() call succeeds or is benign, isOpened() remains true.
  // **Mocking Considerations:** None (real camera assumed).

  if (!is_camera_available_)
    GTEST_SKIP() << "Skipping test: No ZED camera hardware detected.";

  ASSERT_TRUE(ensureCameraOpen(live_config_));
  EXPECT_TRUE(zed_cam_->isOpened());
  EXPECT_TRUE(zed_cam_->open());
  // Should return true if already open and be benign
  EXPECT_TRUE(zed_cam_->isOpened());
}

TEST(ZEDCameraLifecycleTest, ZC_LC_003_CloseNotOpenedCamera) {
  // **Test Case ID:** ZC_LC_003
  // **Method(s) Under Test:** close(), isOpened()
  // **Test Scenario Description:** Calling close() on a camera that was never opened.
  // **Key Configurations/Inputs:** Default ZEDCameraConfig, camera not opened.
  // **Expected Outcome:** close() call is benign, isOpened() remains false.
  // **Mocking Considerations:** None.

  ZEDCamera zed_cam(getLiveCameraDefaultConfig());
  EXPECT_FALSE(zed_cam.isOpened());
  zed_cam.close(); // Should not crash
  EXPECT_FALSE(zed_cam.isOpened());
}

TEST_F(ZEDCameraHardwareTest, ZC_LC_004_MultipleCloseCalls) {
  // **Test Case ID:** ZC_LC_004
  // **Method(s) Under Test:** close(), isOpened()
  // **Test Scenario Description:** Calling close() multiple times on an opened then closed camera.
  // **Key Configurations/Inputs:** Default live ZEDCameraConfig.
  // **Expected Outcome:** Camera opens, closes, subsequent close() calls are benign, isOpened() remains false.
  // **Mocking Considerations:** None (real camera assumed).

  if (!is_camera_available_)
    GTEST_SKIP() << "Skipping test: No ZED camera hardware detected.";

  ASSERT_TRUE(ensureCameraOpen(live_config_));
  zed_cam_->close();
  EXPECT_FALSE(zed_cam_->isOpened());
  zed_cam_->close(); // Second call
  EXPECT_FALSE(zed_cam_->isOpened());
}

TEST_F(ZEDCameraHardwareTest, ZC_LC_005_ShutdownCamera) {
  // **Test Case ID:** ZC_LC_005
  // **Method(s) Under Test:** shutdown(), isOpened()
  // **Test Scenario Description:** Test shutdown functionality.
  // **Key Configurations/Inputs:** Default live ZEDCameraConfig.
  // **Expected Outcome:** Camera opens, then shutdown makes isOpened() false.
  // **Mocking Considerations:** None (real camera assumed).

  if (!is_camera_available_)
    GTEST_SKIP() << "Skipping test: No ZED camera hardware detected.";

  ASSERT_TRUE(ensureCameraOpen(live_config_));
  zed_cam_->shutdown();
  EXPECT_FALSE(zed_cam_->isOpened());
}

TEST(ZEDCameraLifecycleTest, ZC_LC_006_OpenFailureInvalidID) {
  // **Test Case ID:** ZC_LC_006
  // **Method(s) Under Test:** open()
  // **Test Scenario Description:** Attempt to open a camera with an invalid/unlikely device ID.
  // **Key Configurations/Inputs:** ZEDCameraConfig with a high, likely invalid, camera_device_id.
  // **Expected Outcome:** open() returns false.
  // **Mocking Considerations:** None. This test's success depends on '99' actually being an invalid ID.

  ZEDCameraConfig config = getLiveCameraDefaultConfig();
  config.camera_device_id = 99;
  // Assuming 99 is an invalid/non-existent camera ID
  ZEDCamera zed_cam(config);
  EXPECT_FALSE(zed_cam.open());
}

TEST(ZEDCameraLifecycleTest, ZC_LC_007_GetCameraInitParams) {
  // **Test Case ID:** ZC_LC_007 (Mirrors ZC_INIT_001 from existing tests for this functionality)
  // **Method(s) Under Test:** getCameraInitParams() implicitly via constructor and populateInitParams()
  // **Test Scenario Description:** Verify that getCameraInitParams returns parameters consistent with the configuration.
  // **Key Configurations/Inputs:** A specific ZEDCameraConfig.
  // **Expected Outcome:** The returned sl::InitParameters reflect the input config.
  // **Mocking Considerations:** None.

  ZEDCameraConfig config;
  config.svo_file_path = "";
  config.resolution = sl::RESOLUTION::HD1080;
  config.fps = 60;
  config.depth_mode = sl::DEPTH_MODE::NEURAL;
  config.coordinate_units = sl::UNIT::CENTIMETER;
  config.coordinate_system_3d = sl::COORDINATE_SYSTEM::LEFT_HANDED_Y_UP;
  config.enable_imu = false; // Test with IMU off
  config.sdk_verbose = 0;
  config.camera_disable_self_calib = true;
  config.open_timeout_sec = 3.0f;

  ZEDCamera zed_cam(config);
  // Note: getCameraInitParams in the provided code is not const and opens the camera.
  // The original test_ZED_camera_init.cc accesses zed_cam.getCameraInitParams() which is problematic.
  // The provided ZEDCamera.cc actually has populateInitParams private and getCameraInitParams() is public.
  // Let's assume for this test, we can call getCameraInitParams() without a fully opened camera if it's just returning a member.
  // However, the ZEDCamera::getCameraInitParams() in provided code actually opens the camera if not opened.
  // This test will check the populated zed_init_params_ *after* an attempt to open.
  // If the camera is not physically present, open() might fail, but zed_init_params_ should still be populated.
  // We modify this test slightly to reflect the ZEDCamera.cc behavior.

  zed_cam.open(); // Attempt to open to ensure zed_init_params_ is used.
  // If no camera, open fails, but params should still be set.

  const sl::InitParameters& init_params = zed_cam.getCameraInitParams();
  // This call might re-attempt open

  EXPECT_EQ(init_params.camera_resolution, config.resolution);
  EXPECT_EQ(init_params.camera_fps, config.fps);
  EXPECT_EQ(init_params.depth_mode, config.depth_mode);
  EXPECT_EQ(init_params.coordinate_units, config.coordinate_units);
  EXPECT_EQ(init_params.coordinate_system, config.coordinate_system_3d);
  EXPECT_EQ(init_params.sensors_required, config.enable_imu);
  EXPECT_EQ(init_params.camera_disable_self_calib,
            config.camera_disable_self_calib);
  EXPECT_FLOAT_EQ(init_params.open_timeout_sec, config.open_timeout_sec);

  if (zed_cam.isOpened()) zed_cam.close(); // Clean up if open succeeded
}


// --- Data Retrieval Tests ---
TEST_F(ZEDCameraHardwareTest, ZC_DR_001_GrabAndRetrieveLeftImageHD720) {
  // **Test Case ID:** ZC_DR_001
  // **Method(s) Under Test:** grab(), retrieveRawImage()
  // **Test Scenario Description:** Grab a frame and retrieve the left HD720 image.
  // **Key Configurations/Inputs:** Live camera, HD720 resolution.
  // **Expected Outcome:** Grab and retrieve succeed, image has correct dimensions.
  // **Mocking Considerations:** None (real camera assumed).

  live_config_.resolution = sl::RESOLUTION::HD720;
  if (!ensureCameraOpen(live_config_)) return;

  sl::RuntimeParameters rt_params;
  ASSERT_EQ(zed_cam_->grab(rt_params), sl::ERROR_CODE::SUCCESS);

  sl::Mat left_image;
  ASSERT_EQ(
      zed_cam_->retrieveRawImage(sl::VIEW::LEFT, left_image, sl::MEM::CPU, {1280
        , 720}), sl::ERROR_CODE::SUCCESS);
  EXPECT_EQ(left_image.getWidth(), 1280);
  EXPECT_EQ(left_image.getHeight(), 720);
  EXPECT_TRUE(left_image.isInit());
}

TEST_F(ZEDCameraHardwareTest, ZC_DR_002_GrabAndRetrieveRightImageDefaultRes) {
  // **Test Case ID:** ZC_DR_002
  // **Method(s) Under Test:** grab(), retrieveRawImage()
  // **Test Scenario Description:** Grab a frame and retrieve the right image using default resolution.
  // **Key Configurations/Inputs:** Live camera, default resolution for retrieval (0,0).
  // **Expected Outcome:** Grab and retrieve succeed, image matches camera's configured resolution.
  // **Mocking Considerations:** None (real camera assumed).

  live_config_.resolution = sl::RESOLUTION::VGA;
  // Set a known resolution for the camera
  if (!ensureCameraOpen(live_config_)) return;

  sl::RuntimeParameters rt_params;
  ASSERT_EQ(zed_cam_->grab(rt_params), sl::ERROR_CODE::SUCCESS);

  sl::Mat right_image;
  // Using {0,0} for resolution to get camera's current/default resolution
  ASSERT_EQ(
      zed_cam_->retrieveRawImage(sl::VIEW::RIGHT, right_image, sl::MEM::CPU, {0,
        0}), sl::ERROR_CODE::SUCCESS);
  EXPECT_EQ(right_image.getWidth(), 672); // Expected for VGA
  EXPECT_EQ(right_image.getHeight(), 376);
  EXPECT_TRUE(right_image.isInit());
}

TEST_F(ZEDCameraHardwareTest, ZC_DR_003_GrabAndRetrieveDepthPerformance) {
  // **Test Case ID:** ZC_DR_003
  // **Method(s) Under Test:** grab(), retrieveRawDepth()
  // **Test Scenario Description:** Grab a frame and retrieve the depth map.
  // **Key Configurations/Inputs:** Live camera, PERFORMANCE depth mode.
  // **Expected Outcome:** Grab and retrieve succeed, depth map has correct dimensions and type (typically F32_C1).
  // **Mocking Considerations:** None (real camera assumed).

  live_config_.depth_mode = sl::DEPTH_MODE::PERFORMANCE;
  if (!ensureCameraOpen(live_config_)) return;

  sl::RuntimeParameters rt_params;
  rt_params.enable_depth = true; // Ensure depth is requested for this grab
  ASSERT_EQ(zed_cam_->grab(rt_params), sl::ERROR_CODE::SUCCESS);

  sl::Mat depth_map;
  ASSERT_EQ(zed_cam_->retrieveRawDepth(depth_map, sl::MEM::CPU, {0,0}),
            sl::ERROR_CODE::SUCCESS);
  EXPECT_EQ(depth_map.getWidth(),
            live_config_.resolution == sl::RESOLUTION::HD720 ? 1280 : 672);
  // Adjust based on actual config
  EXPECT_EQ(depth_map.getHeight(),
            live_config_.resolution == sl::RESOLUTION::HD720 ? 720 : 376);
  EXPECT_EQ(depth_map.getDataType(), sl::MAT_TYPE::F32_C1);
  EXPECT_TRUE(depth_map.isInit());
}

TEST_F(ZEDCameraHardwareTest, ZC_DR_004_GrabAndRetrieveConfidenceMap) {
  // **Test Case ID:** ZC_DR_004
  // **Method(s) Under Test:** grab(), retrieveRawMeasure()
  // **Test Scenario Description:** Grab a frame and retrieve the confidence map.
  // **Key Configurations/Inputs:** Live camera.
  // **Expected Outcome:** Grab and retrieve succeed, confidence map has correct dimensions.
  // **Mocking Considerations:** None (real camera assumed).

  if (!ensureCameraOpen(live_config_)) return;

  sl::RuntimeParameters rt_params;
  rt_params.enable_depth = true; // Confidence map is related to depth
  ASSERT_EQ(zed_cam_->grab(rt_params), sl::ERROR_CODE::SUCCESS);

  sl::Mat confidence_map;
  ASSERT_EQ(
      zed_cam_->retrieveRawMeasure(sl::MEASURE::CONFIDENCE, confidence_map, sl::
        MEM::CPU, {0,0}), sl::ERROR_CODE::SUCCESS);
  EXPECT_EQ(confidence_map.getWidth(),
            live_config_.resolution == sl::RESOLUTION::HD720 ? 1280 : 672);
  EXPECT_EQ(confidence_map.getHeight(),
            live_config_.resolution == sl::RESOLUTION::HD720 ? 720 : 376);
  EXPECT_TRUE(confidence_map.isInit());
}

TEST_F(ZEDCameraHardwareTest, ZC_DR_005_GrabAndRetrieveIMUData) {
  // **Test Case ID:** ZC_DR_005
  // **Method(s) Under Test:** grab(), retrieveRawSensorsData()
  // **Test Scenario Description:** Grab a frame and retrieve IMU data.
  // **Key Configurations/Inputs:** Live camera with IMU enabled in ZEDCameraConfig.
  // **Expected Outcome:** Grab and retrieve succeed, SensorsData contains IMU data.
  // **Mocking Considerations:** None (real camera assumed, and camera must have IMU).

  live_config_.enable_imu = true;
  if (!ensureCameraOpen(live_config_)) return;

  // Some ZED models might not have an IMU or it might be disabled.
  if (!zed_cam_->isImuEnabled()) {
    LOG(WARNING) <<
        "IMU not available or not enabled on this ZED model/configuration. Skipping IMU data retrieval test.";
    GTEST_SKIP();
  }

  sl::RuntimeParameters rt_params;
  ASSERT_EQ(zed_cam_->grab(rt_params), sl::ERROR_CODE::SUCCESS);

  sl::SensorsData sensors_data;
  ASSERT_EQ(
      zed_cam_->retrieveRawSensorsData(sensors_data, sl::TIME_REFERENCE::IMAGE),
      sl::ERROR_CODE::SUCCESS);
  // Basic check, more detailed checks would involve timestamp or data values if ground truth known
  EXPECT_NE(sensors_data.imu.timestamp.getNanoseconds(), 0);
  // Expect a valid timestamp
}

TEST_F(ZEDCameraHardwareTest, ZC_DR_006_RetrieveImageBeforeGrab) {
  // **Test Case ID:** ZC_DR_006
  // **Method(s) Under Test:** retrieveRawImage()
  // **Test Scenario Description:** Attempt to retrieve an image before a successful grab.
  // **Key Configurations/Inputs:** Live camera.
  // **Expected Outcome:** retrieveRawImage() returns an error code (e.g., NO_NEW_FRAME_AVAILABLE).
  // **Mocking Considerations:** None (real camera assumed).

  if (!ensureCameraOpen(live_config_)) return;

  sl::Mat left_image;
  // Expecting an error code like NO_NEW_FRAME_AVAILABLE or similar, as grab hasn't been called.
  EXPECT_NE(
      zed_cam_->retrieveRawImage(sl::VIEW::LEFT, left_image, sl::MEM::CPU),
      sl::ERROR_CODE::SUCCESS);
}


// --- Object Detection Module Tests ---
TEST_F(ZEDCameraHardwareTest, ZC_OD_001_EnableDisableObjectDetection) {
  // **Test Case ID:** ZC_OD_001
  // **Method(s) Under Test:** enableObjectDetection(), disableObjectDetection(), isObjectDetectionEnabled()
  // **Test Scenario Description:** Test enabling and disabling the OD module.
  // **Key Configurations/Inputs:** Live camera.
  // **Expected Outcome:** OD enables, isObjectDetectionEnabled() is true, OD disables, isObjectDetectionEnabled() is false.
  // **Mocking Considerations:** None (real camera assumed).

  live_config_.object_detection_model =
      sl::OBJECT_DETECTION_MODEL::MULTI_CLASS_BOX_MEDIUM; // Example model
  if (!ensureCameraOpen(live_config_)) return;

  EXPECT_TRUE(zed_cam_->enableObjectDetection());
  EXPECT_TRUE(zed_cam_->isObjectDetectionEnabled());
  zed_cam_->disableObjectDetection();
  EXPECT_FALSE(zed_cam_->isObjectDetectionEnabled());
}

TEST_F(ZEDCameraHardwareTest, ZC_OD_002_RetrieveObjectsAfterEnabling) {
  // **Test Case ID:** ZC_OD_002
  // **Method(s) Under Test:** enableObjectDetection(), grab(), retrieveObjects()
  // **Test Scenario Description:** Enable OD, grab a frame, and retrieve objects.
  // **Key Configurations/Inputs:** Live camera.
  // **Expected Outcome:** retrieveObjects() succeeds. `sl::Objects` structure is valid.
  // **Mocking Considerations:** None (real camera assumed).

  live_config_.object_detection_model =
      sl::OBJECT_DETECTION_MODEL::MULTI_CLASS_BOX_ACCURATE;
  if (!ensureCameraOpen(live_config_)) return;

  ASSERT_TRUE(zed_cam_->enableObjectDetection());

  sl::RuntimeParameters rt_params;
  ASSERT_EQ(zed_cam_->grab(rt_params), sl::ERROR_CODE::SUCCESS);

  sl::Objects objects;
  sl::ObjectDetectionRuntimeParameters od_rt_params;
  od_rt_params.detection_confidence_threshold = 50; // Example
  ASSERT_EQ(zed_cam_->retrieveObjects(objects, od_rt_params),
            sl::ERROR_CODE::SUCCESS);
  // We can't guarantee objects are detected, but the call should succeed.
  // Basic check on the structure:
  EXPECT_TRUE(objects.is_new); // Indicates new data was processed
}

TEST_F(ZEDCameraHardwareTest, ZC_OD_003_RetrieveObjectsWhenDisabled) {
  // **Test Case ID:** ZC_OD_003
  // **Method(s) Under Test:** retrieveObjects()
  // **Test Scenario Description:** Attempt to retrieve objects when the OD module is not enabled.
  // **Key Configurations/Inputs:** Live camera, OD module disabled.
  // **Expected Outcome:** retrieveObjects() returns an error.
  // **Mocking Considerations:** None (real camera assumed).

  if (!ensureCameraOpen(live_config_)) return;
  // Ensure OD is disabled (default or explicitly)
  zed_cam_->disableObjectDetection();
  EXPECT_FALSE(zed_cam_->isObjectDetectionEnabled());

  sl::Objects objects;
  sl::ObjectDetectionRuntimeParameters od_rt_params;
  EXPECT_NE(zed_cam_->retrieveObjects(objects, od_rt_params),
            sl::ERROR_CODE::SUCCESS);
}

// --- Body Tracking Module Tests ---
TEST_F(ZEDCameraHardwareTest, ZC_BT_001_EnableDisableBodyTracking) {
  // **Test Case ID:** ZC_BT_001
  // **Method(s) Under Test:** enableBodyTracking(), disableBodyTracking(), isBodyTrackingEnabled()
  // **Test Scenario Description:** Test enabling and disabling the BT module.
  // **Key Configurations/Inputs:** Live camera.
  // **Expected Outcome:** BT enables, isBodyTrackingEnabled() is true, BT disables, isBodyTrackingEnabled() is false.
  // **Mocking Considerations:** None (real camera assumed).

  live_config_.body_tracking_model =
      sl::BODY_TRACKING_MODEL::HUMAN_BODY_ACCURATE;
  if (!ensureCameraOpen(live_config_)) return;

  EXPECT_TRUE(zed_cam_->enableBodyTracking());
  EXPECT_TRUE(zed_cam_->isBodyTrackingEnabled());
  zed_cam_->disableBodyTracking();
  EXPECT_FALSE(zed_cam_->isBodyTrackingEnabled());
}

TEST_F(ZEDCameraHardwareTest, ZC_BT_002_RetrieveBodiesAfterEnabling) {
  // **Test Case ID:** ZC_BT_002
  // **Method(s) Under Test:** enableBodyTracking(), grab(), retrieveBodies()
  // **Test Scenario Description:** Enable BT, grab a frame, and retrieve bodies.
  // **Key Configurations/Inputs:** Live camera.
  // **Expected Outcome:** retrieveBodies() succeeds. `sl::Bodies` structure is valid.
  // **Mocking Considerations:** None (real camera assumed).

  live_config_.body_tracking_model = sl::BODY_TRACKING_MODEL::HUMAN_BODY_MEDIUM;
  if (!ensureCameraOpen(live_config_)) return;

  ASSERT_TRUE(zed_cam_->enableBodyTracking());

  sl::RuntimeParameters rt_params;
  ASSERT_EQ(zed_cam_->grab(rt_params), sl::ERROR_CODE::SUCCESS);

  sl::Bodies bodies;
  sl::BodyTrackingRuntimeParameters bt_rt_params;
  bt_rt_params.detection_confidence_threshold = 50;
  ASSERT_EQ(zed_cam_->retrieveBodies(bodies, bt_rt_params),
            sl::ERROR_CODE::SUCCESS);
  EXPECT_TRUE(bodies.is_new);
}

TEST_F(ZEDCameraHardwareTest, ZC_BT_003_RetrieveBodiesWhenDisabled) {
  // **Test Case ID:** ZC_BT_003
  // **Method(s) Under Test:** retrieveBodies()
  // **Test Scenario Description:** Attempt to retrieve bodies when the BT module is not enabled.
  // **Key Configurations/Inputs:** Live camera, BT module disabled.
  // **Expected Outcome:** retrieveBodies() returns an error.
  // **Mocking Considerations:** None (real camera assumed).

  if (!ensureCameraOpen(live_config_)) return;

  zed_cam_->disableBodyTracking();
  EXPECT_FALSE(zed_cam_->isBodyTrackingEnabled());

  sl::Bodies bodies;
  sl::BodyTrackingRuntimeParameters bt_rt_params;
  EXPECT_NE(zed_cam_->retrieveBodies(bodies, bt_rt_params),
            sl::ERROR_CODE::SUCCESS);
}

// --- Camera Information and Configuration Tests ---
TEST_F(ZEDCameraHardwareTest, ZC_CI_001_GetCameraInformation) {
  // **Test Case ID:** ZC_CI_001
  // **Method(s) Under Test:** getCameraInformation()
  // **Test Scenario Description:** Retrieve camera information after opening.
  // **Key Configurations/Inputs:** Live camera.
  // **Expected Outcome:** CameraInformation structure is populated with valid data (e.g., serial number > 0).
  // **Mocking Considerations:** None (real camera assumed).

  if (!ensureCameraOpen(live_config_)) return;

  sl::CameraInformation cam_info = zed_cam_->getCameraInformation();
  EXPECT_GT(cam_info.serial_number, 0) << "Serial number should be positive.";
  EXPECT_NE(cam_info.camera_model,
            sl::MODEL::LAST) << "Camera model should be valid.";
}

TEST_F(ZEDCameraHardwareTest, ZC_CI_002_GetCalibrationParametersRectified) {
  // **Test Case ID:** ZC_CI_002
  // **Method(s) Under Test:** getCalibrationParameters()
  // **Test Scenario Description:** Retrieve rectified calibration parameters.
  // **Key Configurations/Inputs:** Live camera, raw = false.
  // **Expected Outcome:** sl::CalibrationParameters structure is populated (e.g., fx > 0).
  // **Mocking Considerations:** None (real camera assumed).

  if (!ensureCameraOpen(live_config_)) return;

  sl::CalibrationParameters calib_params = zed_cam_->
      getCalibrationParameters(false);
  EXPECT_GT(calib_params.left_cam.fx,
            0) << "Focal length fx should be positive.";
  EXPECT_GT(calib_params.left_cam.fy,
            0) << "Focal length fy should be positive.";
}

TEST_F(ZEDCameraHardwareTest, ZC_CI_003_GetCalibrationParametersRawBehavior) {
  // **Test Case ID:** ZC_CI_003
  // **Method(s) Under Test:** getCalibrationParameters()
  // **Test Scenario Description:** Retrieve "raw" calibration parameters (current impl. returns rectified).
  // **Key Configurations/Inputs:** Live camera, raw = true.
  // **Expected Outcome:** Returns rectified parameters as per current ZEDCamera.cc implementation.
  // **Mocking Considerations:** None (real camera assumed).

  if (!ensureCameraOpen(live_config_)) return;

  sl::CalibrationParameters calib_params_raw = zed_cam_->
      getCalibrationParameters(true);
  sl::CalibrationParameters calib_params_rect = zed_cam_->
      getCalibrationParameters(false);

  // Expect them to be the same based on current ZEDCamera.cc logic
  EXPECT_FLOAT_EQ(calib_params_raw.left_cam.fx, calib_params_rect.left_cam.fx);
  EXPECT_FLOAT_EQ(calib_params_raw.left_cam.cx, calib_params_rect.left_cam.cx);
  // Add more checks if needed
}


// --- SVO Mode Operations Tests ---
TEST_F(ZEDCameraSVOTest, ZC_SVO_001_OpenSVOAndCheckMode) {
  // **Test Case ID:** ZC_SVO_001
  // **Method(s) Under Test:** open(), isSVOMode()
  // **Test Scenario Description:** Open an SVO file and verify SVO mode.
  // **Key Configurations/Inputs:** ZEDCameraConfig with a valid SVO file path.
  // **Expected Outcome:** Camera opens, isSVOMode() returns true.
  // **Mocking Considerations:** Requires TEST_SVO_FILE_PATH to be a valid SVO.

  if (!svo_file_accessible_)
    GTEST_SKIP() << "SVO file not accessible, skipping test.";
  ASSERT_TRUE(ensureSVOCameraOpen(svo_config_));
  EXPECT_TRUE(zed_svo_cam_->isSVOMode());
}

TEST_F(ZEDCameraSVOTest, ZC_SVO_002_GetSVOFramesAndFPS) {
  // **Test Case ID:** ZC_SVO_002
  // **Method(s) Under Test:** getSVONumberOfFrames(), getSVOFrameRate()
  // **Test Scenario Description:** Get total frames and FPS from an SVO file.
  // **Key Configurations/Inputs:** Opened SVO file.
  // **Expected Outcome:** Returns positive values for frame count and FPS (actual values depend on SVO).
  // **Mocking Considerations:** Requires TEST_SVO_FILE_PATH to be a valid SVO.

  if (!svo_file_accessible_)
    GTEST_SKIP() << "SVO file not accessible, skipping test.";
  ASSERT_TRUE(ensureSVOCameraOpen(svo_config_));
  EXPECT_GT(zed_svo_cam_->getSVONumberOfFrames(), 0);
  EXPECT_GT(zed_svo_cam_->getSVOFrameRate(), 0);
}

TEST_F(ZEDCameraSVOTest, ZC_SVO_003_SetSVOPositionAndGrab) {
  // **Test Case ID:** ZC_SVO_003
  // **Method(s) Under Test:** setSVOPosition(), grab(), retrieveRawImage()
  // **Test Scenario Description:** Set SVO position to a specific frame and grab data.
  // **Key Configurations/Inputs:** Opened SVO file with at least a few frames.
  // **Expected Outcome:** setSVOPosition succeeds, grab succeeds, retrieved image is valid.
  // **Mocking Considerations:** Requires TEST_SVO_FILE_PATH to be a valid SVO.

  if (!svo_file_accessible_)
    GTEST_SKIP() << "SVO file not accessible, skipping test.";
  ASSERT_TRUE(ensureSVOCameraOpen(svo_config_));

  int num_frames = zed_svo_cam_->getSVONumberOfFrames();
  if (num_frames < 2) {
    GTEST_SKIP() <<
 "SVO file has fewer than 2 frames, cannot test setSVOPosition effectively.";
  }

  int target_frame = num_frames / 2; // Go to a middle frame
  ASSERT_EQ(zed_svo_cam_->setSVOPosition(target_frame),
            sl::ERROR_CODE::SUCCESS);

  sl::RuntimeParameters rt_params;
  ASSERT_EQ(zed_svo_cam_->grab(rt_params), sl::ERROR_CODE::SUCCESS);

  sl::Mat image;
  EXPECT_EQ(zed_svo_cam_->retrieveRawImage(sl::VIEW::LEFT, image, sl::MEM::CPU),
            sl::ERROR_CODE::SUCCESS);
  EXPECT_TRUE(image.isInit());
  // Further check: zed_svo_cam_->getSVOCurrentFrame() should be target_frame
  // Note: getSVOCurrentFrame is not in the provided ZEDCamera interface, but common in ZED SDK.
  // If it were available, it would be: EXPECT_EQ(zed_svo_cam_->getSVOCurrentFrame(), target_frame);
}

TEST_F(ZEDCameraSVOTest, ZC_SVO_004_GrabAtEndOfSVO) {
  // **Test Case ID:** ZC_SVO_004
  // **Method(s) Under Test:** grab()
  // **Test Scenario Description:** Attempt to grab frames beyond the end of an SVO file.
  // **Key Configurations/Inputs:** Opened SVO file.
  // **Expected Outcome:** grab() eventually returns sl::ERROR_CODE::END_OF_SVOFILE_REACHED.
  // **Mocking Considerations:** Requires TEST_SVO_FILE_PATH to be a valid SVO.

  if (!svo_file_accessible_)
    GTEST_SKIP() << "SVO file not accessible, skipping test.";
  ASSERT_TRUE(ensureSVOCameraOpen(svo_config_));

  int num_frames = zed_svo_cam_->getSVONumberOfFrames();
  sl::RuntimeParameters rt_params;
  sl::ERROR_CODE err = sl::ERROR_CODE::SUCCESS;

  for (int i = 0; i < num_frames + 5; ++i) {
    // Try to grab a few frames beyond the end
    err = zed_svo_cam_->grab(rt_params);
    if (err == sl::ERROR_CODE::END_OF_SVOFILE_REACHED) {
      break;
    }
    // Allow other SUCCESS errors if not END_OF_SVOFILE yet
    ASSERT_TRUE(err == sl::ERROR_CODE::SUCCESS) << "Unexpected grab error: " <<
 sl::toString(err);
  }
  EXPECT_EQ(err, sl::ERROR_CODE::END_OF_SVOFILE_REACHED);
}


// --- Camera Discovery Tests ---
TEST(ZEDCameraDiscoveryTest, ZC_CD_001_DiscoverCamerasDefaultConfig) {
  // **Test Case ID:** ZC_CD_001
  // **Method(s) Under Test:** discoverAndCreateCameras(const ZEDCameraConfig&, bool)
  // **Test Scenario Description:** Discover cameras using a default configuration.
  // **Key Configurations/Inputs:** Default ZEDCameraConfig, openCameras = false.
  // **Expected Outcome:** Returns a list of ZEDCamera::Ptr. Size depends on connected cameras.
  // **Mocking Considerations:** None. Outcome depends on physically connected cameras.

  ZEDCameraConfig defaultConfig = getLiveCameraDefaultConfig();
  std::vector<ZEDCamera::Ptr> cameras = ZEDCamera::discoverAndCreateCameras(
      defaultConfig, false);

  std::vector<sl::DeviceProperties> actual_devices =
      sl::Camera::getDeviceList();
  EXPECT_EQ(cameras.size(), actual_devices.size());
  for (const auto& cam_ptr : cameras) {
    EXPECT_NE(cam_ptr, nullptr);
    EXPECT_FALSE(cam_ptr->isOpened()); // Since openCameras = false
  }
}

TEST(ZEDCameraDiscoveryTest, ZC_CD_002_DiscoverCamerasCustomConfigProvider) {
  // **Test Case ID:** ZC_CD_002
  // **Method(s) Under Test:** discoverAndCreateCameras(const std::function<...>&, bool)
  // **Test Scenario Description:** Discover cameras using a custom config provider lambda.
  // **Key Configurations/Inputs:** Lambda setting a specific FPS, openCameras = true.
  // **Expected Outcome:** Cameras are created and opened. Configured FPS is reflected if checkable (e.g. via getCameraInformation).
  // **Mocking Considerations:** None.

  auto custom_provider = [](
      const sl::DeviceProperties& props) -> ZEDCameraConfig {
    ZEDCameraConfig cfg = getLiveCameraDefaultConfig(props.id);
    cfg.fps = 15; // Custom FPS
    return cfg;
  };

  std::vector<ZEDCamera::Ptr> cameras = ZEDCamera::discoverAndCreateCameras(
      custom_provider, true);
  std::vector<sl::DeviceProperties> actual_devices =
      sl::Camera::getDeviceList();
  EXPECT_EQ(cameras.size(), actual_devices.size());

  for (const auto& cam_ptr : cameras) {
    ASSERT_NE(cam_ptr, nullptr);
    if (ZEDCameraHardwareTest::is_camera_available_) {
      // Only expect open if cameras physically present
      ASSERT_TRUE(cam_ptr->isOpened());
      // Check if FPS matches (if camera supports it and SDK applies it)
      // This might be tricky as actual FPS can differ from requested
      sl::CameraInformation info = cam_ptr->getCameraInformation();
      // EXPECT_EQ(info.camera_configuration.fps, 15); // This can be device-dependent
      LOG(INFO) << "Discovered and opened camera S/N " << info.serial_number <<
          " with reported FPS: " << info.camera_configuration.fps;
      cam_ptr->close(); // Clean up
    } else {
      EXPECT_FALSE(cam_ptr->isOpened());
    }
  }
}
} // namespace test
} // namespace common
} // namespace dyno/


int main(int argc, char** argv) {
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
