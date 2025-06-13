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