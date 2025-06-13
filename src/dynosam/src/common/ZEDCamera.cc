// common/ZEDCamera.cc
#include "dynosam/common/ZEDCamera.hpp"

namespace dyno {
namespace common {
ZEDCamera::ZEDCamera(const ZEDCameraConfig& config)
  : config_(config) {
  populateInitParams();
}

ZEDCamera::~ZEDCamera() {
  shutdown();
}


std::vector<ZEDCamera::Ptr> ZEDCamera::discoverAndCreateCameras(
    const std::function<ZEDCameraConfig(const sl::DeviceProperties&)>& configProvider,
    bool openCameras) {
    std::vector<ZEDCamera::Ptr> result_cameras;
    std::vector<sl::DeviceProperties> device_list = sl::Camera::getDeviceList();

    LOG(INFO) << "Discovering ZED cameras. Found " << device_list.size() << " ZED device(s).";

    for (const auto& props : device_list) {
        LOG(INFO) << "Processing ZED camera with S/N: " << props.serial_number
                  << ", ID: " << props.id
                  << ", Model: " << sl::toString(props.camera_model).c_str();

        ZEDCameraConfig config = configProvider(props); // The configProvider is responsible for setting camera_device_id correctly.

        ZEDCamera::Ptr camera = std::make_shared<ZEDCamera>(config);

        if (openCameras) {
            if (camera->open()) {
                LOG(INFO) << "Successfully opened ZED camera with S/N: " << props.serial_number << " and ID: " << props.id;
                result_cameras.push_back(camera);
            } else {
                LOG(ERROR) << "Failed to open ZED camera with S/N: " << props.serial_number << " and ID: " << props.id;
                // Do not add to the list if open fails
            }
        } else {
            LOG(INFO) << "Configured ZED camera with S/N: " << props.serial_number << " and ID: " << props.id << ". Not opening as per request.";
            result_cameras.push_back(camera);
        }
    }
    return result_cameras;
}

std::vector<ZEDCamera::Ptr> ZEDCamera::discoverAndCreateCameras(
    const ZEDCameraConfig& defaultConfig,
    bool openCameras) {

    auto default_config_provider = [&](const sl::DeviceProperties& props) -> ZEDCameraConfig {
        ZEDCameraConfig specific_config = defaultConfig;
        specific_config.camera_device_id = props.id;
        return specific_config;
    };

    return discoverAndCreateCameras(default_config_provider, openCameras);
}

void ZEDCamera::populateInitParams() {
  zed_init_params_.camera_resolution = config_.resolution;
  zed_init_params_.camera_fps = config_.fps;
  zed_init_params_.depth_mode = config_.depth_mode;
  zed_init_params_.coordinate_units = config_.coordinate_units;
  zed_init_params_.coordinate_system = config_.coordinate_system_3d;
  zed_init_params_.sdk_verbose = config_.sdk_verbose > 0;
  zed_init_params_.camera_disable_self_calib = config_.
      camera_disable_self_calib;
  zed_init_params_.enable_image_enhancement = config_.enable_image_enhancement;
  zed_init_params_.open_timeout_sec = config_.open_timeout_sec;
  zed_init_params_.enable_right_side_measure = config_.
      enable_right_side_measure;
  zed_init_params_.async_grab_camera_recovery = config_.
      async_grab_camera_recovery;
  zed_init_params_.grab_compute_capping_fps = config_.grab_compute_capping_fps;
  zed_init_params_.enable_image_validity_check = config_.
      enable_image_validity_check;

  if (!config_.svo_file_path.empty()) {
    zed_init_params_.input.setFromSVOFile(config_.svo_file_path.c_str());
    zed_init_params_.svo_real_time_mode = config_.svo_real_time_mode;
  }
  zed_init_params_.sensors_required = config_.enable_imu;
}

bool ZEDCamera::open() {
  if (is_opened_.load()) {
    LOG(WARNING) << "ZEDCamera::open() called but camera is already open.";
    return true;
  }
  sl::ERROR_CODE err = zed_camera_.open(zed_init_params_);
  if (err != sl::ERROR_CODE::SUCCESS) {
    LOG(ERROR) << "ZEDCamera: Failed to open ZED camera: " << sl::toString(err);
    is_opened_.store(false);
    return false;
  }
  is_opened_.store(true);
  LOG(INFO) << "ZEDCamera: ZED camera opened successfully.";
  return true;
}

void ZEDCamera::close() {
  if (is_opened_.exchange(false)) {
    // Atomically set to false and get previous value
    disableObjectDetection();
    disableBodyTracking();
    zed_camera_.close();
    LOG(INFO) << "ZEDCamera: ZED camera closed.";
  }
}

void ZEDCamera::shutdown() {
  close();
}

bool ZEDCamera::isOpened() const {
  return is_opened_.load() && zed_camera_.isOpened();
}

sl::ERROR_CODE ZEDCamera::grab(const sl::RuntimeParameters& runtime_params) {
  if (!isOpened()) {
    LOG(ERROR) << "ZEDCamera::grab() called but camera is not open.";
    return sl::ERROR_CODE::CAMERA_NOT_INITIALIZED;
  }
  return zed_camera_.grab(runtime_params);
}

sl::ERROR_CODE ZEDCamera::retrieveRawImage(sl::VIEW view_identifier,
                                           sl::Mat& out_image, sl::MEM mem_type,
                                           const sl::Resolution& resolution) {
  if (!isOpened()) return sl::ERROR_CODE::CAMERA_NOT_INITIALIZED;
  // If resolution width/height are 0, ZED SDK uses default resolution.
  return zed_camera_.retrieveImage(out_image, view_identifier, mem_type,
                                   resolution);
}

sl::ERROR_CODE ZEDCamera::retrieveRawDepth(sl::Mat& out_depth, sl::MEM mem_type,
                                           const sl::Resolution& resolution) {
  return retrieveRawMeasure(sl::MEASURE::DEPTH, out_depth, mem_type,
                            resolution);
}

sl::ERROR_CODE ZEDCamera::retrieveRawMeasure(sl::MEASURE measure,
                                             sl::Mat& out_measure,
                                             sl::MEM mem_type,
                                             const sl::Resolution& resolution) {
  if (!isOpened()) return sl::ERROR_CODE::CAMERA_NOT_INITIALIZED;
  return zed_camera_.
      retrieveMeasure(out_measure, measure, mem_type, resolution);
}

sl::ERROR_CODE ZEDCamera::retrieveRawSensorsData(
    sl::SensorsData& out_sensors_data, sl::TIME_REFERENCE time_ref) {
  if (!isOpened()) {
    LOG(ERROR) <<
        "ZEDCamera::retrieveRawSensorsData() called but camera is not open.";
    return sl::ERROR_CODE::CAMERA_NOT_INITIALIZED;
  }
  if (!config_.enable_imu) {
    LOG(WARNING) <<
        "ZEDCamera::retrieveRawSensorsData() called but IMU is not enabled in config.";
    return sl::ERROR_CODE::SENSORS_NOT_AVAILABLE;
  }
  return zed_camera_.getSensorsData(out_sensors_data, time_ref);
}

bool ZEDCamera::enableObjectDetection() {
  if (!isOpened()) {
    LOG(ERROR) <<
        "ZEDCamera::enableObjectDetection() called but camera is not open.";
    return false;
  }
  if (is_object_detection_enabled_.load()) {
    LOG(WARNING) <<
        "ZEDCamera::enableObjectDetection() called but OD is already enabled.";
    return true;
  }

  sl::ObjectDetectionParameters od_params;
  od_params.detection_model = config_.object_detection_model;
  od_params.enable_tracking = config_.object_detection_enable_tracking;
  od_params.enable_segmentation = config_.object_detection_enable_segmentation;
  // od_params.image_sync = true; // Default behavior, consider if explicit setting is needed

  sl::ERROR_CODE err = zed_camera_.enableObjectDetection(od_params);
  if (err != sl::ERROR_CODE::SUCCESS) {
    LOG(ERROR) << "ZEDCamera: Failed to enable Object Detection: " <<
        sl::toString(err);
    is_object_detection_enabled_.store(false);
    return false;
  }
  is_object_detection_enabled_.store(true);
  LOG(INFO) << "ZEDCamera: Object Detection enabled.";
  return true;
}

void ZEDCamera::disableObjectDetection() {
  if (is_opened_.load() && is_object_detection_enabled_.exchange(false)) {
    zed_camera_.disableObjectDetection();
    LOG(INFO) << "ZEDCamera: Object Detection disabled.";
  }
}

sl::ERROR_CODE ZEDCamera::retrieveObjects(sl::Objects& out_objects,
                                          const
                                          sl::ObjectDetectionRuntimeParameters&
                                          params) {
  if (!isOpened()) return sl::ERROR_CODE::CAMERA_NOT_INITIALIZED;
  if (!is_object_detection_enabled_.load()) {
    LOG(WARNING) <<
        "ZEDCamera::retrieveObjects() called but OD is not enabled.";
    return sl::ERROR_CODE::MODULE_NOT_COMPATIBLE_WITH_CAMERA;
  }
  return zed_camera_.retrieveObjects(out_objects, params);
}

bool ZEDCamera::enableBodyTracking() {
  if (!isOpened()) {
    LOG(ERROR) <<
        "ZEDCamera::enableBodyTracking() called but camera is not open.";
    return false;
  }
  if (is_body_tracking_enabled_.load()) {
    LOG(WARNING) <<
        "ZEDCamera::enableBodyTracking() called but BT is already enabled.";
    return true;
  }

  sl::BodyTrackingParameters bt_params;
  bt_params.detection_model = config_.body_tracking_model;
  bt_params.body_format = config_.body_format;
  bt_params.enable_tracking = config_.object_detection_enable_tracking;
  // Reuse general tracking flag
  bt_params.enable_body_fitting = config_.body_tracking_enable_fitting;
  bt_params.enable_segmentation = config_.body_tracking_enable_segmentation;
  bt_params.allow_reduced_precision_inference = config_.
      body_tracking_allow_reduced_precision_inference;
  // bt_params.image_sync = true;

  sl::ERROR_CODE err = zed_camera_.enableBodyTracking(bt_params);
  if (err != sl::ERROR_CODE::SUCCESS) {
    LOG(ERROR) << "ZEDCamera: Failed to enable Body Tracking: " <<
        sl::toString(err);
    is_body_tracking_enabled_.store(false);
    return false;
  }
  is_body_tracking_enabled_.store(true);
  LOG(INFO) << "ZEDCamera: Body Tracking enabled.";
  return true;
}

void ZEDCamera::disableBodyTracking() {
  if (is_opened_.load() && is_body_tracking_enabled_.exchange(false)) {
    zed_camera_.disableBodyTracking();
    LOG(INFO) << "ZEDCamera: Body Tracking disabled.";
  }
}

sl::ERROR_CODE ZEDCamera::retrieveBodies(sl::Bodies& out_bodies,
                                         const sl::BodyTrackingRuntimeParameters
                                         & params) {
  if (!isOpened()) return sl::ERROR_CODE::CAMERA_NOT_INITIALIZED;
  if (!is_body_tracking_enabled_.load()) {
    LOG(WARNING) << "ZEDCamera::retrieveBodies() called but BT is not enabled.";
    return sl::ERROR_CODE::MODULE_NOT_COMPATIBLE_WITH_CAMERA;
  }
  return zed_camera_.retrieveBodies(out_bodies, params);
}

sl::CameraInformation ZEDCamera::getCameraInformation() const {
  if (!isOpened()) {
    LOG(ERROR) <<
        "ZEDCamera::getCameraInformation() called but camera is not open. Returning default.";
    return sl::CameraInformation();
  }
  return zed_camera_.getCameraInformation();
}

sl::CalibrationParameters ZEDCamera::getCalibrationParameters(bool raw) const {
  if (!isOpened()) {
    LOG(ERROR) <<
        "ZEDCamera::getCalibrationParameters() called but camera is not open. Returning default.";
    return {};
  }
  if (raw) {
    // This is a bit of a simplification. The "raw" parameters might be part of a different substruct
    // or require different handling depending on ZED SDK version and specific needs.
    // Typically, ZED provides rectified images, so `camera_configuration.calibration_parameters` is for those.
    // Accessing truly "raw" (pre-rectification) might need specific SDK features if available.
    // For now, this illustrates the intent.
#if ZED_SDK_MAJOR_VERSION >= 3 && ZED_SDK_MINOR_VERSION >= 8 // Example, check actual SDK versioning for raw params


    // return getCameraInformation().camera_configuration.calibration_parameters_raw; // Hypothetical
#endif
    LOG(WARNING) <<
        "Raw calibration parameters requested but might not be distinctly available or handled in this simplified version. Returning rectified.";
  }
  return getCameraInformation().camera_configuration.calibration_parameters;
  // Returns rectified by default
}

sl::InitParameters ZEDCamera::getCameraInitParams()  {
  if (!isOpened()) {
    LOG(ERROR) << "ZEDCamera::getCameraParams() called but camera is not open. Returning default.";
    return {};
  }
  // Return the left camera parameters (rectified)
  return zed_init_params_;
}

bool ZEDCamera::isSVOMode() const {
  return !config_.svo_file_path.empty();
}

int ZEDCamera::getSVONumberOfFrames() {
  if (!isOpened() || !isSVOMode()) {
    return 0;
  }
  return zed_camera_.getSVONumberOfFrames();
}

unsigned int ZEDCamera::getSVOFrameRate() const {
  if (!isOpened() || !isSVOMode()) {
    return 0;
  }
  // The SVO's original FPS is stored in CameraInformation after opening.
  return getCameraInformation().camera_configuration.fps;
}

sl::ERROR_CODE ZEDCamera::setSVOPosition(int frame_position) {
  if (!isOpened() || !isSVOMode()) {
    return isSVOMode()
             ? sl::ERROR_CODE::CAMERA_NOT_INITIALIZED
             : sl::ERROR_CODE::MODULE_NOT_COMPATIBLE_WITH_CAMERA;
  }
  zed_camera_.setSVOPosition(frame_position);
  return sl::ERROR_CODE::SUCCESS;
}
} // namespace common
} // namespace dyno
