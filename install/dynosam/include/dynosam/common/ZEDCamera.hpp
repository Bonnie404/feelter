// common/ZEDCamera.h
#ifndef DYNO_COMMON_ZEDCAMERA_HPP_
#define DYNO_COMMON_ZEDCAMERA_HPP_

#include "dynosam/common/Types.hpp"
#include "dynosam/common/ZEDCameraConfig.hpp"
#include <sl/Camera.hpp>
#include <string>
#include <atomic>
#include <optional>

namespace dyno {
namespace common {
class ZEDCamera {
public:
  DYNO_POINTER_TYPEDEFS(ZEDCamera)

  explicit ZEDCamera(const ZEDCameraConfig& config);
  ~ZEDCamera();

  ZEDCamera(const ZEDCamera&) = delete;
  ZEDCamera& operator=(const ZEDCamera&) = delete;
  ZEDCamera(ZEDCamera&&) = delete;
  ZEDCamera& operator=(ZEDCamera&&) = delete;

  bool open();
  void close();
  bool isOpened() const;
  void shutdown();

  /**
* @brief Discovers all connected ZED cameras and attempts to open them.
* @param configProvider A function that takes sl::DeviceProperties of a detected camera
*                       and returns a ZEDCameraConfig to be used for that specific camera.
*                       The camera_device_id within the returned ZEDCameraConfig MUST be
*                       set correctly using sl::DeviceProperties::id by the provider.
* @param openCameras If true (default), the method will attempt to open each discovered camera.
*                    If false, it will return a list of ZEDCamera objects configured but not yet opened.
* @return A vector of ZEDCamera::Ptr objects. If openCameras is true, these are successfully opened cameras.
*         If openCameras is false, these are configured but unopened cameras.
*         If a camera fails to open (when openCameras is true), it is omitted from the list.
*/
  static std::vector<ZEDCamera::Ptr> discoverAndCreateCameras(
      const std::function<ZEDCameraConfig(const sl::DeviceProperties&)>&
      configProvider,
      bool openCameras = true);

  /**
 * @brief Overload that uses a default ZEDCameraConfig for all discovered cameras.
 * The camera_device_id in the defaultConfig will be overridden by the detected device ID.
 * @param defaultConfig The base configuration to use for all cameras.
 * @param openCameras If true (default), the method will attempt to open each discovered camera.
 *                    If false, it will return a list of ZEDCamera objects configured but not yet opened.
 * @return A vector of ZEDCamera::Ptr objects. If openCameras is true, these are successfully opened cameras.
 *         If openCameras is false, these are configured but unopened cameras.
 *         If a camera fails to open (when openCameras is true), it is omitted from the list.
 */
  static std::vector<ZEDCamera::Ptr> discoverAndCreateCameras(
      const ZEDCameraConfig& defaultConfig = ZEDCameraConfig(),
      bool openCameras = false);

  sl::ERROR_CODE grab(const sl::RuntimeParameters& runtime_params);

  sl::ERROR_CODE retrieveRawImage(sl::VIEW view_identifier, sl::Mat& out_image,
                                  sl::MEM mem_type = sl::MEM::CPU,
                                  const sl::Resolution& resolution =
                                      sl::Resolution(0, 0));
  sl::ERROR_CODE retrieveRawDepth(sl::Mat& out_depth,
                                  sl::MEM mem_type = sl::MEM::CPU,
                                  const sl::Resolution& resolution =
                                      sl::Resolution(0, 0));
  sl::ERROR_CODE retrieveRawMeasure(sl::MEASURE measure, sl::Mat& out_measure,
                                    sl::MEM mem_type = sl::MEM::CPU,
                                    const sl::Resolution& resolution =
                                        sl::Resolution(0, 0));

  bool enableObjectDetection();
  void disableObjectDetection();
  bool isObjectDetectionEnabled() const { return is_object_detection_enabled_; }
  sl::ERROR_CODE retrieveObjects(sl::Objects& out_objects,
                                 const sl::ObjectDetectionRuntimeParameters&
                                 params);

  bool enableBodyTracking();
  void disableBodyTracking();
  bool isBodyTrackingEnabled() const { return is_body_tracking_enabled_; }
  sl::ERROR_CODE retrieveBodies(sl::Bodies& out_bodies,
                                const sl::BodyTrackingRuntimeParameters&
                                params);

  bool isImuEnabled() const { return config_.enable_imu; }
  sl::ERROR_CODE retrieveRawSensorsData(sl::SensorsData& out_sensors_data,
                                        sl::TIME_REFERENCE time_ref =
                                            sl::TIME_REFERENCE::IMAGE);

  sl::CameraInformation getCameraInformation() const;
  // Gets rectified calibration parameters by default
  sl::CalibrationParameters getCalibrationParameters(bool raw = false) const;
  sl::InitParameters getCameraInitParams();

  bool isSVOMode() const;
  int getSVONumberOfFrames();
  unsigned int getSVOFrameRate() const;              // Added for completeness
  sl::ERROR_CODE setSVOPosition(int frame_position); // Added for completeness

private:
  sl::Camera zed_camera_;
  ZEDCameraConfig config_;
  sl::InitParameters zed_init_params_;
  std::atomic<bool> is_opened_ = false;
  std::atomic<bool> is_object_detection_enabled_ = false;
  std::atomic<bool> is_body_tracking_enabled_ = false;

  void populateInitParams();
};
} // namespace common
} // namespace dyno

#endif // DYNO_COMMON_ZEDCAMERA_HPP_
