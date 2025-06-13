// dynosam/dataprovider/ZED2DataProvider.cc

#include "dynosam/dataprovider/ZEDDataProvider.hpp"
#include "dynosam/common/ImageTypes.hpp"
#include "dynosam/utils/OpenCVUtils.hpp"
#include "dynosam/utils/GtsamUtils.hpp"

#include <glog/logging.h>

namespace dyno {

// Helper function moved inside the .cc or to a common utility if ZED2Config is widely used for this
common::ZEDCameraConfig ZEDDataProvider::createZEDCameraConfig(const ZEDConfig& dyno_config) {
    common::ZEDCameraConfig zed_cam_config;
    zed_cam_config.resolution = dyno_config.resolution;
    zed_cam_config.fps = dyno_config.fps;
    zed_cam_config.depth_mode = dyno_config.depth_mode;
    zed_cam_config.coordinate_units = dyno_config.coordinate_units;
    zed_cam_config.coordinate_system_3d = dyno_config.coordinate_system_3d_zed; // Use ZED specific coord system
    zed_cam_config.svo_file_path = dyno_config.svo_file_path;
    zed_cam_config.svo_real_time_mode = dyno_config.svo_real_time_mode;
    zed_cam_config.enable_imu = dyno_config.enable_imu;
    zed_cam_config.enable_right_side_measure = dyno_config.enable_right_side_measure;
    zed_cam_config.sdk_verbose = dyno_config.sdk_verbose_level;
    zed_cam_config.camera_device_id = dyno_config.camera_device_id;
    zed_cam_config.camera_disable_self_calib = dyno_config.camera_disable_self_calib;
    zed_cam_config.enable_image_enhancement = dyno_config.enable_image_enhancement;
    zed_cam_config.open_timeout_sec = dyno_config.open_timeout_sec;
    zed_cam_config.enable_image_validity_check = false;


    // Module specific params
    zed_cam_config.object_detection_model = dyno_config.object_detection_model;
    zed_cam_config.object_detection_enable_tracking = dyno_config.object_detection_enable_tracking;
    zed_cam_config.object_detection_enable_segmentation = dyno_config.object_detection_enable_segmentation;

    zed_cam_config.body_tracking_model = dyno_config.body_tracking_model;
    zed_cam_config.body_format = dyno_config.body_format;
    zed_cam_config.body_tracking_enable_fitting = dyno_config.body_tracking_enable_fitting;
    zed_cam_config.body_tracking_enable_segmentation = dyno_config.body_tracking_enable_segmentation;
    zed_cam_config.body_tracking_allow_reduced_precision_inference = dyno_config.body_tracking_allow_reduced_precision_inference;

    return zed_cam_config;
}

ZEDDataProvider::ZEDDataProvider(const ZEDConfig& config,
                                     DataInterfacePipeline* data_interface_pipeline)
    : DataProvider(data_interface_pipeline),
      config_(config) {
    zed_camera_wrapper_ = std::make_shared<common::ZEDCamera>(createZEDCameraConfig(config_));
    if (!initializeZED()) {
        LOG(FATAL) << "ZED2DataProvider: Failed to initialize ZED camera system.";
    }
}

ZEDDataProvider::ZEDDataProvider(const ZEDConfig& config,
                                   DataInterfacePipelineImu* data_interface_pipeline_imu)
    : DataProvider(data_interface_pipeline_imu),
      config_(config) {
    zed_camera_wrapper_ = std::make_shared<common::ZEDCamera>(createZEDCameraConfig(config_));
    if (!initializeZED()) {
        LOG(FATAL) << "ZED2DataProvider: Failed to initialize ZED camera system.";
    }
}

ZEDDataProvider::~ZEDDataProvider() {
    ZEDDataProvider::shutdown();
}

bool ZEDDataProvider::initializeZED() {
    CHECK_NOTNULL(zed_camera_wrapper_);
    if (!zed_camera_wrapper_->open()) {
        return false;
    }

    zed_runtime_params_.enable_depth = true; // Depth is fundamental for RGBD

    setupDynoSAMCameraParams();
    if (!camera_params_) {
        LOG(ERROR) << "ZED2DataProvider: Failed to setup dynosam CameraParams.";
        zed_camera_wrapper_->shutdown();
        return false;
    }

    if (config_.enable_object_detection) {
        if (!zed_camera_wrapper_->enableObjectDetection()) {
            LOG(ERROR) << "ZED2DataProvider: Failed to enable Object Detection module.";
            // Decide on graceful degradation or failure
        }
    }

    if (config_.enable_body_tracking) {
        if (!zed_camera_wrapper_->enableBodyTracking()) {
            LOG(ERROR) << "ZED2DataProvider: Failed to enable Body Tracking module.";
        }
    }

    LOG(INFO) << "ZED2DataProvider: ZED system initialized successfully.";
    return true;
}

void ZEDDataProvider::setupDynoSAMCameraParams() {
    CHECK_NOTNULL(zed_camera_wrapper_);
    sl::CameraInformation cam_info = zed_camera_wrapper_->getCameraInformation();
    sl::CalibrationParameters calib_params_left = cam_info.camera_configuration.calibration_parameters; // Rectified Left Cam


    sl::Resolution res = cam_info.camera_configuration.resolution;

    CameraParams::IntrinsicsCoeffs intrinsics = {
        calib_params_left.left_cam.fx,
        calib_params_left.left_cam.fy,
        calib_params_left.left_cam.cx,
        calib_params_left.left_cam.cy
    };

    // ZED usually provides rectified images, meaning distortion is handled.
    // For dynosam::CameraParams, pass zero distortion coefficients.
    CameraParams::DistortionCoeffs distortion_coeffs;
    distortion_coeffs.assign(5, 0.0); // Assuming plumb_bob/radtan with 5 zero coeffs

    // The model type still matters for some GTSAM calibrators.
    std::string distortion_model_str = "radial_tangential";
    DistortionModel distortion_model_enum = CameraParams::stringToDistortion(distortion_model_str, "pinhole");

    gtsam::Pose3 T_BS = gtsam::Pose3::Identity();
    if (config_.enable_imu && cam_info.sensors_configuration.isSensorAvailable(sl::SENSOR_TYPE::ACCELEROMETER)) {
        // T_BS is transform from body (IMU) to sensor (Left Camera Optical Frame)
        // ZED SDK provides camera_imu_transform: T_LeftCamera_ImuCenter
        sl::Transform zed_T_camL_imu = cam_info.sensors_configuration.camera_imu_transform;
        gtsam::Rot3 R_camL_imu(zed_T_camL_imu.getRotationMatrix().r00,
                               zed_T_camL_imu.getRotationMatrix().r01,
                               zed_T_camL_imu.getRotationMatrix().r02,
                               zed_T_camL_imu.getRotationMatrix().r10,
                               zed_T_camL_imu.getRotationMatrix().r11,
                               zed_T_camL_imu.getRotationMatrix().r12,
                               zed_T_camL_imu.getRotationMatrix().r20,
                               zed_T_camL_imu.getRotationMatrix().r21,
                               zed_T_camL_imu.getRotationMatrix().r22);
        gtsam::Point3 T_camL_imu(zed_T_camL_imu.getTranslation().x,
                                         zed_T_camL_imu.getTranslation().y,
                                         zed_T_camL_imu.getTranslation().z);
        T_BS = gtsam::Pose3(R_camL_imu, T_camL_imu).inverse();
    }

    camera_params_ = CameraParams(intrinsics,
                                  distortion_coeffs,
                                  cv::Size(static_cast<int>(res.width), static_cast<int>(res.height)),
                                  distortion_model_enum,
                                  T_BS);
}

cv::Mat ZEDDataProvider::slMat_to_cvMat(const sl::Mat& sl_mat) {
    // Implementation remains the same as it's a generic utility.
    if (sl_mat.getMemoryType() == sl::MEM::GPU) {
         LOG(FATAL) << "GPU sl::Mat to cv::Mat conversion not directly supported without CPU copy.";
    }
    int cv_type = -1;
    switch (sl_mat.getDataType()) {
        case sl::MAT_TYPE::F32_C1: cv_type = CV_32FC1; break;
        case sl::MAT_TYPE::F32_C2: cv_type = CV_32FC2; break;
        case sl::MAT_TYPE::F32_C3: cv_type = CV_32FC3; break;
        case sl::MAT_TYPE::F32_C4: cv_type = CV_32FC4; break;
        case sl::MAT_TYPE::U8_C1:  cv_type = CV_8UC1;  break;
        case sl::MAT_TYPE::U8_C2:  cv_type = CV_8UC2;  break;
        case sl::MAT_TYPE::U8_C3:  cv_type = CV_8UC3;  break;
        case sl::MAT_TYPE::U8_C4:  cv_type = CV_8UC4;  break;
        default: LOG(FATAL) << "Unsupported sl::MAT_TYPE for cv::Mat conversion: " << sl_mat.getDataType();
    }
    return cv::Mat(sl_mat.getHeight(), sl_mat.getWidth(), cv_type,
                   sl_mat.getPtr<sl::uchar1>(sl::MEM::CPU), sl_mat.getStepBytes(sl::MEM::CPU));
}

ImageContainer::Ptr ZEDDataProvider::createImageContainer(
    Timestamp timestamp,
    FrameId frame_id,
    const sl::Mat& left_sl,
    const sl::Mat& depth_sl,
    const std::optional<sl::Objects>& objects,
    const std::optional<sl::Bodies>& bodies) const {
    // Implementation remains largely the same, focuses on data conversion and ImageContainer creation.
    cv::Mat left_cv_native = slMat_to_cvMat(left_sl);
    cv::Mat left_cv_dynosam;

    if (config_.output_rgb) {
        if (left_cv_native.channels() == 4) cv::cvtColor(left_cv_native, left_cv_dynosam, cv::COLOR_BGRA2BGR);
        else if (left_cv_native.channels() == 3) left_cv_dynosam = left_cv_native.clone();
        else if (left_cv_native.channels() == 1) cv::cvtColor(left_cv_native, left_cv_dynosam, cv::COLOR_GRAY2BGR);
        else LOG(FATAL) << "Unsupported channel count for RGB: " << left_cv_native.channels();
    } else {
        if (left_cv_native.channels() == 4) cv::cvtColor(left_cv_native, left_cv_dynosam, cv::COLOR_BGRA2GRAY);
        else if (left_cv_native.channels() == 3) cv::cvtColor(left_cv_native, left_cv_dynosam, cv::COLOR_BGR2GRAY);
        else if (left_cv_native.channels() == 1) left_cv_dynosam = left_cv_native.clone();
        else LOG(FATAL) << "Unsupported channel count for Grayscale: " << left_cv_native.channels();
    }
    ImageWrapper<ImageType::RGBMono> rgb_mono_wrapper(left_cv_dynosam);

    cv::Mat depth_cv_native = slMat_to_cvMat(depth_sl);
    cv::Mat depth_cv_dynosam;
    if (depth_cv_native.type() == CV_32FC1) {
        depth_cv_native.convertTo(depth_cv_dynosam, CV_64F);
    } else {
        LOG(ERROR) << "ZED Depth map is not CV_32FC1: " << utils::cvTypeToString(depth_cv_native.type());
        depth_cv_native.convertTo(depth_cv_dynosam, CV_64F);
    }
    ImageWrapper<ImageType::Depth> depth_wrapper(depth_cv_dynosam);

    ImageWrapper<ImageType::OpticalFlow> optical_flow_wrapper; // ZED doesn't provide this directly; DynoSAM calculates it.
    ImageWrapper<ImageType::MotionMask> motion_mask_wrapper;
    ImageWrapper<ImageType::ClassSegmentation> class_segmentation_wrapper; // Not directly from ZED, unless using custom models.

    if (config_.enable_object_detection || config_.enable_body_tracking) {
        ImageType::MotionMask dummy_type_indicator;
        cv::Mat generated_mask = generateMaskFromZEDDetections(objects, bodies, rgb_mono_wrapper.image.size(), dummy_type_indicator);
        if (!generated_mask.empty()) {
            motion_mask_wrapper = ImageWrapper<ImageType::MotionMask>(generated_mask);
        }
    }

    return ImageContainer::Create(timestamp, frame_id, rgb_mono_wrapper, depth_wrapper,
                                  optical_flow_wrapper, motion_mask_wrapper, class_segmentation_wrapper);
}

cv::Mat ZEDDataProvider::generateMaskFromZEDDetections(
    const std::optional<sl::Objects>& zed_objects,
    const std::optional<sl::Bodies>& zed_bodies,
    const cv::Size& image_size,
    ImageType::MotionMask& /*mask_type_indicator*/) const {
    // Implementation remains the same.
    cv::Mat combined_mask = cv::Mat::zeros(image_size, CV_32SC1); // DynoSAM expects CV_32SC1 for masks

    if (config_.enable_object_detection && zed_objects.has_value()) {
        for (const auto& obj : zed_objects->object_list) {
            if (config_.object_detection_enable_segmentation && obj.mask.isInit() && !obj.mask.getInfos().empty()) {
                cv::Mat obj_cv_mask = slMat_to_cvMat(obj.mask);
                for (int r = 0; r < obj_cv_mask.rows; ++r) {
                    for (int c = 0; c < obj_cv_mask.cols; ++c) {
                        if (obj_cv_mask.at<unsigned char>(r,c) > 0) {
                             if (r < combined_mask.rows && c < combined_mask.cols) {
                                combined_mask.at<int32_t>(r,c) = static_cast<int32_t>(obj.id);
                            }
                        }
                    }
                }
            } else { // Fallback to bounding box
                int min_x = static_cast<int>(std::min({obj.bounding_box_2d[0].x, obj.bounding_box_2d[3].x}));
                int max_x = static_cast<int>(std::max({obj.bounding_box_2d[1].x, obj.bounding_box_2d[2].x}));
                int min_y = static_cast<int>(std::min({obj.bounding_box_2d[0].y, obj.bounding_box_2d[1].y}));
                int max_y = static_cast<int>(std::max({obj.bounding_box_2d[2].y, obj.bounding_box_2d[3].y}));
                cv::Rect roi(cv::Point(min_x, min_y), cv::Point(max_x, max_y));
                roi &= cv::Rect(0,0, image_size.width, image_size.height); // Intersect with image bounds
                if(roi.area() > 0) combined_mask(roi).setTo(static_cast<int32_t>(obj.id));
            }
        }
    }

    if (config_.enable_body_tracking && zed_bodies.has_value()) {
        for (const auto& body : zed_bodies->body_list) {
            ObjectId body_mask_id = static_cast<ObjectId>(body.id);
            if (config_.enable_object_detection) body_mask_id += 1000; // Offset

            if (config_.body_tracking_enable_segmentation && body.mask.isInit() && !body.mask.getInfos().empty()){
                 cv::Mat body_cv_mask = slMat_to_cvMat(body.mask);
                 for (int r = 0; r < body_cv_mask.rows; ++r) {
                    for (int c = 0; c < body_cv_mask.cols; ++c) {
                        if (body_cv_mask.at<unsigned char>(r,c) > 0) {
                             if (r < combined_mask.rows && c < combined_mask.cols) {
                                combined_mask.at<int32_t>(r,c) = body_mask_id;
                            }
                        }
                    }
                }
            } else { // Fallback to bounding box for bodies
                int min_x = static_cast<int>(std::min({body.bounding_box_2d[0].x, body.bounding_box_2d[3].x}));
                int max_x = static_cast<int>(std::max({body.bounding_box_2d[1].x, body.bounding_box_2d[2].x}));
                int min_y = static_cast<int>(std::min({body.bounding_box_2d[0].y, body.bounding_box_2d[1].y}));
                int max_y = static_cast<int>(std::max({body.bounding_box_2d[2].y, body.bounding_box_2d[3].y}));
                cv::Rect roi(cv::Point(min_x, min_y), cv::Point(max_x, max_y));
                roi &= cv::Rect(0,0, image_size.width, image_size.height); // Intersect
                if(roi.area() > 0) combined_mask(roi).setTo(body_mask_id);
            }
        }
    }
    return combined_mask;
}

bool ZEDDataProvider::spin() {
    if (is_shutting_down_.load() || !zed_camera_wrapper_ || !zed_camera_wrapper_->isOpened()) {
        return false;
    }

    sl::ERROR_CODE err = zed_camera_wrapper_->grab(zed_runtime_params_);
    if (err == sl::ERROR_CODE::SUCCESS) {
        sl::Mat left_image_sl, depth_map_sl;
        // Retrieve left image (BGRA typically)
        zed_camera_wrapper_->retrieveRawImage(sl::VIEW::LEFT, left_image_sl, sl::MEM::CPU);
        // Retrieve depth map (F32_C1)
        zed_camera_wrapper_->retrieveRawDepth(depth_map_sl, sl::MEM::CPU);

        Timestamp timestamp_ns = left_image_sl.timestamp.getNanoseconds();
        Timestamp timestamp_s = static_cast<double>(timestamp_ns) * 1e-9;


        std::optional<sl::Objects> zed_objects_opt = std::nullopt;
        if (config_.enable_object_detection && zed_camera_wrapper_->isObjectDetectionEnabled()) {
            sl::Objects objects_data;
            sl::ObjectDetectionRuntimeParameters od_rt_params;
            od_rt_params.detection_confidence_threshold = config_.detection_confidence_threshold_od;
            if (zed_camera_wrapper_->retrieveObjects(objects_data, od_rt_params) == sl::ERROR_CODE::SUCCESS) {
                zed_objects_opt = objects_data;
            }
        }

        std::optional<sl::Bodies> zed_bodies_opt = std::nullopt;
        if (config_.enable_body_tracking && zed_camera_wrapper_->isBodyTrackingEnabled()) {
            sl::Bodies bodies_data;
            sl::BodyTrackingRuntimeParameters bt_rt_params;
            bt_rt_params.detection_confidence_threshold = config_.detection_confidence_threshold_bt;
            if (zed_camera_wrapper_->retrieveBodies(bodies_data, bt_rt_params) == sl::ERROR_CODE::SUCCESS) {
                zed_bodies_opt = bodies_data;
            }
        }

        ImageContainer::Ptr image_container = createImageContainer(
            timestamp_s, frame_id_counter_, left_image_sl, depth_map_sl, zed_objects_opt, zed_bodies_opt);

        CHECK(image_container_callback_);
        image_container_callback_(image_container);

        if (config_.enable_imu && zed_camera_wrapper_->isImuEnabled()) {
            if (imu_single_input_callback_) {
                sl::SensorsData sensors_data;
                if (zed_camera_wrapper_->retrieveRawSensorsData(sensors_data, sl::TIME_REFERENCE::IMAGE) == sl::ERROR_CODE::SUCCESS) {
                    ImuMeasurement imu_measurement;
                    imu_measurement.timestamp_ = static_cast<double>(sensors_data.imu.timestamp.getNanoseconds()) * 1e-9;
                    // ZED IMU (NED): X Fwd, Y Right, Z Down
                    // DynoSAM (FLU assumed): X Fwd, Y Left, Z Up
                    imu_measurement.acc_gyr_ << sensors_data.imu.linear_acceleration.x,    // Acc X
                                               -sensors_data.imu.linear_acceleration.y,   // Acc Y
                                               -sensors_data.imu.linear_acceleration.z,   // Acc Z
                                                sensors_data.imu.angular_velocity.x,     // Gyro X (rad/s)
                                               -sensors_data.imu.angular_velocity.y,     // Gyro Y (rad/s)
                                               -sensors_data.imu.angular_velocity.z;     // Gyro Z (rad/s)
                    imu_single_input_callback_(imu_measurement);
                }
            } else {
                LOG_EVERY_N(WARNING, 100) << "IMU processing requested, but DataInterfacePipeline does not support IMU callbacks.";
            }
        }

        frame_id_counter_++;
        return true;
    } else if (err == sl::ERROR_CODE::END_OF_SVOFILE_REACHED) {
        LOG(INFO) << "ZED2DataProvider: End of SVO file reached.";
        shutdown();
        return false;
    } else {
        LOG(ERROR) << "ZED2DataProvider: ZEDCamera::grab() failed: " << sl::toString(err);
        shutdown();
        return false;
    }
}

void ZEDDataProvider::shutdown() {
    if (is_shutting_down_.exchange(true) == false) {
        LOG(INFO) << "ZED2DataProvider: Shutting down...";
        if (zed_camera_wrapper_) {
            zed_camera_wrapper_->shutdown();
        }
        DataProvider::shutdown();
    }
}

CameraParams::Optional ZEDDataProvider::getCameraParams() const {
    return camera_params_;
}

int ZEDDataProvider::datasetSize() const {
    if (zed_camera_wrapper_ && zed_camera_wrapper_->isSVOMode()) {
        return zed_camera_wrapper_->getSVONumberOfFrames();
    }
    return -1; // Live camera indication
}

} // namespace dyno