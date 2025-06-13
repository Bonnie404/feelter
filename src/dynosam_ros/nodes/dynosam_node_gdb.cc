// nodes/dynosam_node_gdb.cc
// (Initially a copy of nodes/dynosam_node.cc)

#include <gflags/gflags.h>
#include <glog/logging.h>

#include "dynosam_ros/PipelineRos.hpp"
#include "dynosam_ros/Utils.hpp" // For initRosAndLogging
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"

#include <vector>
#include <string>
#include <iostream> // For std::cerr
#include <cstdlib>  // For std::getenv

// This gflag is from the original dynosam_node.cc
DEFINE_bool(show_dyno_args, false,
            "Show all loaded DynoSAM args (YAML and gflag) and exit");

// Helper function to check if a gflag is already set (to avoid overriding if passed via cmd line)
bool is_gflag_set(const char* name) {
    google::CommandLineFlagInfo info;
    return google::GetCommandLineFlagInfo(name, &info) && !info.is_default;
}

std::vector<std::string> split_string(const std::string& s, char delimiter) {
  std::vector<std::string> tokens;
  std::string token;
  std::istringstream tokenStream(s);
  while (std::getline(tokenStream, token, delimiter)) {
    tokens.push_back(token);
  }
  return tokens;
}

int main(int argc, char* argv[]) {
  std::vector<std::string> effective_args_str;
  effective_args_str.emplace_back(argv[0]); // Program name

  if (!is_gflag_set("v")) {
    effective_args_str.emplace_back("--v=30");
  }
  if (!is_gflag_set("output_path")) {
    effective_args_str.emplace_back("--output_path=/tmp/dynosam_zed_output");
  }

  for (int i = 1; i < argc; ++i) {
    effective_args_str.emplace_back(argv[i]);
  }

  std::vector<char*> effective_argv_c;
  for (const auto& s : effective_args_str) {
    effective_argv_c.push_back(const_cast<char*>(s.c_str()));
  }
  int effective_argc = effective_argv_c.size();
  char** effective_argv = effective_argv_c.data();

  std::vector<std::string> non_ros_args = dyno::initRosAndLogging(effective_argc, effective_argv);

  // Add any --flagfile arguments if necessary and not already set.
  // Example (you'd need to make `dynosam_params_share_dir_for_flags` robust):
  // std::string dynosam_params_share_dir_for_flags = "/path/to/your_ws/install/dynosam/share/dynosam/params/";
  // if (!is_gflag_set("flagfile")) { // This simplistic check assumes only one flagfile or specific flagfile names
  //    effective_args_str.push_back("--flagfile=" + dynosam_params_share_dir_for_flags + "your_config1.flags");
  //    effective_args_str.push_back("--flagfile=" + dynosam_params_share_dir_for_flags + "your_config2.flags");
  // }


  rclcpp::NodeOptions options;
  options.arguments(non_ros_args);
  options.use_intra_process_comms(true);

  // Define ROS Parameters (parameter_overrides) for ZED launch
  std::vector<rclcpp::Parameter> ros_params_override;

  ros_params_override.emplace_back("online", true);
  ros_params_override.emplace_back("online_provider_type", "zed");

  ros_params_override.emplace_back("zed.rgb_topic", "/zed2i/zed_node/rgb/image_rect_color");
  ros_params_override.emplace_back("zed.depth_topic", "/zed2i/zed_node/depth/depth_registered");
  ros_params_override.emplace_back("zed.camera_info_topic", "/zed2i/zed_node/rgb/camera_info");
  ros_params_override.emplace_back("zed.imu_topic", "/zed2i/zed_node/imu/data");
  ros_params_override.emplace_back("zed.image_sync_queue_size", 15);
  ros_params_override.emplace_back("zed.image_sync_slop_sec", 0.034);
  ros_params_override.emplace_back("zed.wait_for_camera_info", true);
  ros_params_override.emplace_back("zed.camera_info_timeout_ms", -1);
  ros_params_override.emplace_back("zed.imu_buffer_size", 200);
  ros_params_override.emplace_back("zed.output_rgb", true);
  ros_params_override.emplace_back("zed.enable_imu_processing", true);

  // General parameters
  std::string dynosam_params_share_dir;
  const char* install_prefix_env_cstr = std::getenv("AMENT_PREFIX_PATH");
  if (install_prefix_env_cstr) {
      std::string install_prefix_env(install_prefix_env_cstr);
      std::vector<std::string> paths = split_string(install_prefix_env, ':');
      for (const std::string& p : paths) {
          // We are looking for a path that ends with "/install/dynosam"
          // or is exactly "/path/to/ws/install/dynosam"
          if (p.length() >= strlen("/install/dynosam") &&
              p.substr(p.length() - strlen("/install/dynosam")) == "/install/dynosam") {
              dynosam_params_share_dir = p + "/share/dynosam/params/";
              std::cerr << "[GDB Main] Found dynosam install prefix: " << p << std::endl;
              std::cerr << "[GDB Main] Deduced dynosam_params_share_dir: " << dynosam_params_share_dir << std::endl;
              break;
          }
      }
  }

  if (dynosam_params_share_dir.empty()) {
      std::cerr << "[GDB Main] ERROR: Could not deduce 'dynosam' install path from AMENT_PREFIX_PATH ("
                << (install_prefix_env_cstr ? install_prefix_env_cstr : "NOT SET") << "). "
                << "Please set 'dynosam_params_share_dir' manually in the C++ code, "
                << "or ensure '/your/path/to/install/dynosam' is in AMENT_PREFIX_PATH." << std::endl;
      // Fallback or error. For GDB, you might hardcode or allow override:
      // Example hardcoding for local debug:
      // dynosam_params_share_dir = "/home/bonnie/DynOSAM/install/dynosam/share/dynosam/params/";
      // std::cerr << "[GDB Main] USING HARDCODED dynosam_params_share_dir: " << dynosam_params_share_dir << std::endl;
      if (dynosam_params_share_dir.empty()) return 1; // Or throw if not hardcoded as fallback
  }
  ros_params_override.emplace_back("params_folder_path", dynosam_params_share_dir);

  // ... (other general ros_params_override as before) ...
  ros_params_override.emplace_back("dataset_path", "/root/data/VDO/kitti/kitti/0004");
  ros_params_override.emplace_back("wait_for_camera_params", true);
  ros_params_override.emplace_back("camera_params_timeout", -1);
  ros_params_override.emplace_back("world_frame_id", "world");
  ros_params_override.emplace_back("camera_frame_id", "camera");

  options.parameter_overrides(ros_params_override);

  // The node name "dynosam" is hardcoded in DynoPipelineManagerRos constructor
  auto ros_pipeline = std::make_shared<dyno::DynoPipelineManagerRos>(options);


  if (FLAGS_show_dyno_args) {
    const dyno::DynoParams& params = ros_pipeline->getDynoParams();
    params.printAllParams(true);
    rclcpp::shutdown();
    return 0;
  } else {
    try {
        ros_pipeline->initalisePipeline();
    } catch (const std::exception& e) {
        RCLCPP_FATAL(ros_pipeline->get_logger(), "Exception during pipeline initialization: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }
  }

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(ros_pipeline);

  RCLCPP_INFO(ros_pipeline->get_logger(), "[GDB Main] Starting main spin loop...");
  while (rclcpp::ok()) {
    if (!ros_pipeline->spinOnce()) {
      RCLCPP_INFO(ros_pipeline->get_logger(), "[GDB Main] Pipeline spinOnce returned false. Shutting down.");
      break;
    }
    exec.spin_some();
  }

  RCLCPP_INFO(ros_pipeline->get_logger(), "[GDB Main] Shutting down.");
  ros_pipeline.reset();
  rclcpp::shutdown();
  return 0;
}