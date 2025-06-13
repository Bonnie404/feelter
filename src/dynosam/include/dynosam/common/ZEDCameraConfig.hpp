// common/ZEDCameraConfig.h
#ifndef DYNO_COMMON_ZEDCAMERACONFIG_HPP_
#define DYNO_COMMON_ZEDCAMERACONFIG_HPP_

#include <sl/Camera.hpp>
#include <string>

namespace dyno {
namespace common {

struct ZEDCameraConfig {
    // sl::InitParameters fields
    sl::RESOLUTION resolution = sl::RESOLUTION::AUTO;
    unsigned int fps = 30;
    sl::DEPTH_MODE depth_mode = sl::DEPTH_MODE::NEURAL;
    sl::UNIT coordinate_units = sl::UNIT::METER;
    sl::COORDINATE_SYSTEM coordinate_system_3d = sl::COORDINATE_SYSTEM::IMAGE;
    std::string svo_file_path = "";
    bool svo_real_time_mode = false;
    bool enable_imu = false;
    bool enable_right_side_measure = false;
    float sdk_verbose = 0; // 0: no verbose, 1: verbose
    int camera_device_id = 0;
    bool camera_disable_self_calib = false;
    bool enable_image_enhancement = true;
    float open_timeout_sec = 5.0f;
    bool async_grab_camera_recovery = false;
    int grab_compute_capping_fps = 0;
    bool enable_image_validity_check = false;


    // Parameters for optional ZED modules
    sl::OBJECT_DETECTION_MODEL object_detection_model = sl::OBJECT_DETECTION_MODEL::MULTI_CLASS_BOX_MEDIUM;
    bool object_detection_enable_tracking = true;
    bool object_detection_enable_segmentation = false;

    sl::BODY_TRACKING_MODEL body_tracking_model = sl::BODY_TRACKING_MODEL::HUMAN_BODY_MEDIUM;
    sl::BODY_FORMAT body_format = sl::BODY_FORMAT::BODY_38; // Or BODY_70, BODY_18
    bool body_tracking_enable_fitting = true;
    bool body_tracking_enable_segmentation = false;
    bool body_tracking_allow_reduced_precision_inference = false;

    // Runtime parameters (can be grouped or passed separately)
    // For simplicity, ZED2DataProvider can manage these if they change frequently.
    // If static, they could be part of ZEDCameraConfig.
    // float detection_confidence_threshold_od = 40.0f;
    // float detection_confidence_threshold_bt = 40.0f;
};

} // namespace common
} // namespace dyno

#endif // DYNO_COMMON_ZEDCAMERACONFIG_HPP_
