// dataprovider/ZED2DataProvider.h
#ifndef DYNO_DATAPROVIDER_ZED2DATAPROVIDER_H_
#define DYNO_DATAPROVIDER_ZED2DATAPROVIDER_H_

#include "dynosam/dataprovider/DataProvider.hpp"
#include "dynosam/dataprovider/DataInterfacePipeline.hpp"
#include "dynosam/common/CameraParams.hpp"
#include "dynosam/common/ImageContainer.hpp"
#include "dynosam/common/ZEDCamera.hpp" // New include
#include "dynosam/common/Types.hpp"

#include <sl/Camera.hpp> // For sl enums in ZED2Config
#include <string>
#include <atomic>
#include <optional>

namespace dyno {

// ZED2Config remains largely the same, as it drives both ZEDCameraConfig and DynoSLAM-specific logic
struct ZEDConfig {
    // Parameters for ZEDCameraConfig population
    sl::RESOLUTION resolution = sl::RESOLUTION::HD720; // Example default
    unsigned int fps = 30;
    sl::DEPTH_MODE depth_mode = sl::DEPTH_MODE::NEURAL;
    sl::UNIT coordinate_units = sl::UNIT::METER;
    sl::COORDINATE_SYSTEM coordinate_system_3d_zed = sl::COORDINATE_SYSTEM::IMAGE;
    std::string svo_file_path = "";
    bool svo_real_time_mode = false;
    int camera_device_id = 0;
    bool camera_disable_self_calib = false;
    bool enable_image_enhancement = true;
    float open_timeout_sec = 5.0f;
    bool enable_right_side_measure = false; // For ZEDCameraConfig
    float sdk_verbose_level = 0;          // For ZEDCameraConfig


    // DynoSLAM specific interpretations or module enabling flags
    bool enable_imu = false; // If IMU data should be acquired AND processed by DynoSLAM
    bool output_rgb = true;  // True for BGR/RGB, false for Grayscale in ImageContainer

    bool enable_object_detection = false; // If DynoSLAM should use ZED's OD
    sl::OBJECT_DETECTION_MODEL object_detection_model = sl::OBJECT_DETECTION_MODEL::MULTI_CLASS_BOX_MEDIUM;
    bool object_detection_enable_tracking = true;
    bool object_detection_enable_segmentation = false; // DynoSAM might generate its own masks

    bool enable_body_tracking = false; // If DynoSLAM should use ZED's BT
    sl::BODY_TRACKING_MODEL body_tracking_model = sl::BODY_TRACKING_MODEL::HUMAN_BODY_FAST;
    sl::BODY_FORMAT body_format = sl::BODY_FORMAT::BODY_38;
    bool body_tracking_enable_fitting = true;
    bool body_tracking_enable_segmentation = false;
    bool body_tracking_allow_reduced_precision_inference = false;


    // Runtime parameters for OD/BT (can be set dynamically too)
    float detection_confidence_threshold_od = 40.0f;
    float detection_confidence_threshold_bt = 40.0f;
};


class ZEDDataProvider : public DataProvider {
public:
    DYNO_POINTER_TYPEDEFS(ZEDDataProvider);

    ZEDDataProvider(const ZEDConfig& config,
                     DataInterfacePipeline* data_interface_pipeline);

    ZEDDataProvider(const ZEDConfig& config,
                     DataInterfacePipelineImu* data_interface_pipeline_imu);

    ~ZEDDataProvider() override;

    ZEDDataProvider(const ZEDDataProvider&) = delete;
    ZEDDataProvider& operator=(const ZEDDataProvider&) = delete;
    ZEDDataProvider(ZEDDataProvider&&) = delete;
    ZEDDataProvider& operator=(ZEDDataProvider&&) = delete;

    bool spin() override;
    void shutdown() override;
    CameraParams::Optional getCameraParams() const override;
    int datasetSize() const override;

private:
    bool initializeZED(); // Uses zed_camera_wrapper_
    void setupDynoSAMCameraParams(); // Uses zed_camera_wrapper_
    static common::ZEDCameraConfig createZEDCameraConfig(const ZEDConfig& dyno_config);

    ImageContainer::Ptr createImageContainer(
        Timestamp timestamp,
        FrameId frame_id,
        const sl::Mat& left_sl,
        const sl::Mat& depth_sl,
        const std::optional<sl::Objects>& objects,
        const std::optional<sl::Bodies>& bodies) const;

    static cv::Mat slMat_to_cvMat(const sl::Mat& sl_mat);

    cv::Mat generateMaskFromZEDDetections(
        const std::optional<sl::Objects>& zed_objects,
        const std::optional<sl::Bodies>& zed_bodies,
        const cv::Size& image_size,
        ImageType::MotionMask& mask_type_indicator) const; // Unchanged signature

    ZEDConfig config_;
    common::ZEDCamera::Ptr zed_camera_wrapper_;
    sl::RuntimeParameters zed_runtime_params_; // For sl::Camera::grab

    CameraParams::Optional camera_params_ = std::nullopt;
    FrameId frame_id_counter_ = 0;
    std::atomic<bool> is_shutting_down_ = false;
};

} // namespace dyno

#endif // DYNO_DATAPROVIDER_ZED2DATAPROVIDER_H_