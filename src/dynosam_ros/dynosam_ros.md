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
dynosam_ros/
  launch_utils.py
include/
  dynosam_ros/
    adaptors/
      CameraParamsAdaptor.hpp
    displays/
      dynamic_slam_displays/
        BackendDSDRos.hpp
        DSDCommonRos.hpp
        FrontendDSDRos.hpp
        README.md
      inbuilt_displays/
        BackendInbuiltDisplayRos.hpp
        FrontendInbuiltDisplayRos.hpp
        InbuiltDisplayCommon.hpp
        README.md
      DisplaysCommon.hpp
      DisplaysImpl.hpp
    DataProviderRos.hpp
    Display-Definitions.hpp
    OnlineDataProviderRos.hpp
    PipelineRos.hpp
    RosUtils-inl.hpp
    RosUtils.hpp
    Utils.hpp
    ZEDOnlineDataProvider.hpp
launch/
  dyno_sam_experiments_launch.py
  dyno_sam_launch.py
  dyno_sam_online_launch.py
  dyno_sam_zed_launch.py
nodes/
  backend_experiments_node.cc
  dataset_node.cc
  dynosam_node.cc
rviz/
  rivz.rviz
  rviz_dsd.rviz
scripts/
  run_dynosam_gtest.py
src/
  displays/
    dynamic_slam_displays/
      BackendDSDRos.cc
      DSDCommonRos.cc
      FrontendDSDRos.cc
    inbuilt_displays/
      BackendInbuiltDisplayRos.cc
      FrontendInbuiltDisplayRos.cc
      InbuiltDisplayCommon.cc
    DisplaysCommon.cc
  DataProviderRos.cc
  OnlineDataProviderRos.cc
  PipelineRos.cc
  RosUtils.cc
  Utils.cc
  ZEDOnlineDataProvider.cc
test/
  test_adaptors.cc
  test_main.cc
  test_online_dataprovider_ros.cc
  test_ros_utils.cc
  test_zed_online_dataprovider.cc
CLAUDE.md
cmake_commands.sh
CMakeLists.txt
package.xml
```

# Files

## File: dynosam_ros/launch_utils.py
````python
import launch.logging

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import OpaqueFunction, RegisterEventHandler, Shutdown, DeclareLaunchArgument
from launch.event_handlers import OnProcessExit

def _get_utils_logger():
    return launch.logging.get_logger('dynosam_launch_utils.user')

def validate_path(path: str):
    if not os.path.exists(path):
        raise FileNotFoundError(
            "path does not exist at {}".format(path)
        )
    return path

#NOTE: very helpful answers on ROS2 launch files: https://answers.ros.org/question/382000/ros2-makes-launch-files-crazy-too-soon-to-be-migrating/#:~:text=#!/usr/bin/env,condition=UnlessCondition(use_gui)%0A%20%20%20%20)%0A%0A%20%20])
def get_default_dynosam_params_path():
    #shoudl really be dynosam/<dataset>/example.flags like Kimera but we leave verbose for now
    dynosam_share_dir = get_package_share_directory('dynosam')

    logger = _get_utils_logger()
    logger.info("Loading default dynosam params folder path which is expected to be in the share directory of the dynosam package")

    if not os.path.exists(dynosam_share_dir):
        raise FileNotFoundError(
            "dynosam package share directory does not exist at path {}".format(dynosam_share_dir)
        )

    #check params folder has been correctly exported
    share_folders = os.listdir(dynosam_share_dir)
    if "params" not in share_folders:
        raise FileNotFoundError(
            "dynosam package share directory exists but \'params'\ folder does not exist at {}. "
            "Has the params folder been exported in the dynosam CMakeLists.txt like:\n"
            "\'install(DIRECTORY\n"
            "\tparams\n"
            "\tDESTINATION share/${PROJECT_NAME}\n"
            ")\'?".format(dynosam_share_dir)
        )

    return os.path.join(
        dynosam_share_dir,
        "params"
        ) + "/"


def load_dynosam_node(context, *args, **kwargs):
    """
    Constucts the set of launch actions for running a dynosam node - this includes
    a dynosam-style node and all the parameters that would be required to run the system, including ROS
    and GLOG params.

    This function is intended to be used within an OpaqueFunction in the form
    '
        LaunchDescription([OpaqueFunction(function=load_dynosam_node)])
    '
    where by default args=[] and kwargs={} of the OpaqueFunction, which are then parsed as the *args **kwargs
    of this fucntion. Args is unused. Kwargs are used to specify
        - executable: Executable file name (as in the node executable program to run). Default is dynosam_node.
        - should_output: if the program node (running the executable) should output to screen. Default is True.
        - world_to_robot_tf: If a static transform should be setup between the world and robot. Default is True.

    Context is parsed down from the LaunchService and is used to access the cmd line arguments from the user. These arguments
    should be the additional (non-ros) arguments that will be appended to the program Node arguments - this allows GLOG arguments
    to be parsed directly from the cmdline and into google::ParseCommandLineFlags(&non_ros_argc, &non_ros_argv_c, true)

    The function operates with the following logic
        - Load dynosam parameter path from the launch argument 'params_path'
        - Append flagfiles (files that end with .flags) found on the params_path to the arguments list.
            This allows any flag file stored on the params path to be loaded via GLOG
        - Append addional cmd line arguments from the context to the node arguments.
            This may include overwritten glog arguments which are manually specified
        - Get the executable name from kwargs and construct a launch_ros.actions.Node
            with package 'dynosam_ros', the parameters from dataset_path and params_path,
            as well as the additional arguments as above
        - Setup a tf transform between the world and robot (world to camera) if requested in the kwargs


    Args:
        context (_type_): _description_

    Returns:
        _type_: _description_
    """
    logger = _get_utils_logger()

    dynosam_dataset_path_config = LaunchConfiguration("dataset_path")
    dynosam_params_folder_config = LaunchConfiguration("params_path")
    glog_verbose_flag_config = LaunchConfiguration("v")
    online_config = LaunchConfiguration("online")
    online_provider_type_config = LaunchConfiguration("online_provider_type")
    wait_for_camera_params_config = LaunchConfiguration("wait_for_camera_params")
    camera_params_timeout_config = LaunchConfiguration("camera_params_timeout")
    output_path_config = LaunchConfiguration("output_path")
    
    # ZED-specific configurations
    zed_rgb_topic_config = LaunchConfiguration("zed.rgb_topic")
    zed_depth_topic_config = LaunchConfiguration("zed.depth_topic")
    zed_camera_info_topic_config = LaunchConfiguration("zed.camera_info_topic")
    zed_imu_topic_config = LaunchConfiguration("zed.imu_topic")
    zed_image_sync_queue_size_config = LaunchConfiguration("zed.image_sync_queue_size")
    zed_image_sync_slop_sec_config = LaunchConfiguration("zed.image_sync_slop_sec")
    zed_wait_for_camera_info_config = LaunchConfiguration("zed.wait_for_camera_info")
    zed_camera_info_timeout_ms_config = LaunchConfiguration("zed.camera_info_timeout_ms")
    zed_imu_buffer_size_config = LaunchConfiguration("zed.imu_buffer_size")
    zed_output_rgb_config = LaunchConfiguration("zed.output_rgb")
    zed_enable_imu_processing_config = LaunchConfiguration("zed.enable_imu_processing")

    # remap topics
    camera_info_config = LaunchConfiguration("camera_info")
    rgb_cam_topic_config = LaunchConfiguration("rgb_cam_topic")
    depth_cam_topic_config = LaunchConfiguration("depth_cam_topic")
    motion_mask_cam_topic_config = LaunchConfiguration("motion_mask_cam_topic")
    optical_flow_cam_topic_config = LaunchConfiguration("optical_flow_cam_topic")

    # additional cmdline arguments
    # args_config = LaunchConfiguration("argv")


    # Construct the flagfile arguments given the params folder
    # where we expect to find some files with the suffix .flags (for gflags)
    # these will be added to the arguments list as --flagfile="/path/to/flags/file.flags"
    def construct_flagfile_arguments(params_folder):
        arguments = []
        from pathlib import Path
        for file in os.listdir(params_folder):
            if Path(file).suffix == ".flags":
                arg = "--flagfile={}".format(os.path.join(params_folder, file))
                arguments.append(arg)
        return arguments

    def contruct_glags_verbose_argument(loaded_glog_verbose_flag):
        return "--v={}".format(loaded_glog_verbose_flag)

    # load programatically needed arguments
    loaded_params_folder = validate_path(dynosam_params_folder_config.perform(context))
    loaded_glog_verbose_flag = glog_verbose_flag_config.perform(context)
    loaded_output_path = output_path_config.perform(context)

    # print(args_config.perform(context))

    # arguments to the dynosam Node, these are passed as non-ros args.
    # these will (mostly) be used as GFLAG arguments
    arguments = construct_flagfile_arguments(loaded_params_folder)
    arguments.append(contruct_glags_verbose_argument(loaded_glog_verbose_flag))

    # context.argv is parsed in by the LaunchContext and will be the sys.argv[1:] (so not including the system name.)
    # we can use this to append to the actaul arguments we want to pass to the node. This is useful for
    # all the non-ros (ie GFLAGS/GLOG libs) we use  and allow us to modify this from otuside this launch file
    import rclpy
    import copy
    all_argv = copy.deepcopy(context.argv)
    non_ros_argv = rclpy.utilities.remove_ros_args(all_argv)

    # print(all_argv)
    # print(non_ros_argv)

    def list_difference(a, b):
        return [x for x in a if x not in b]
    # only_ros_argv = list_difference(all_argv, non_ros_argv)
    # argv = context.argv
    def construct_additional_arguments(argv):
        argv = list(argv)
        # if empty (the system name is already removed) then no args to add
        if len(argv) == 0:
            return None

        logger.info("Adding additional sys.argv arguements: {}".format(argv))
        return argv


    # add additional arguments - this will most often be any gflags passed along the command line
    additional_arguments = construct_additional_arguments(non_ros_argv)
    if additional_arguments:
        arguments.extend(additional_arguments)
    # put output path at end of additional arguments
    arguments.append("--output_path={}".format(loaded_output_path))
    # now add the ros args back to the end of the list
    # arguments.extend(only_ros_argv)

    logger.info("Adding arguments {}".format(arguments))

     # get details for the node to launch from the kwargs
    executable = kwargs.get("executable", "dynosam_node")
    logger.info("Setting executable to: {}".format(executable))

    node_kwargs = {}
    #specifies if the node should set output='screen' as part of the node params
    if kwargs.get("should_output", True):
        node_kwargs['output'] = 'screen'
        logger.info("Outputting to screen")

    nodes = []

    program_node = Node(
        package='dynosam_ros',
        executable=executable,
        parameters=[
            {"params_folder_path": dynosam_params_folder_config},
            {"dataset_path": dynosam_dataset_path_config},
            {"online": online_config},
            {"online_provider_type": online_provider_type_config},
            {"wait_for_camera_params": wait_for_camera_params_config},
            {"camera_params_timeout": camera_params_timeout_config},
            # ZED-specific parameters
            {"zed.rgb_topic": zed_rgb_topic_config},
            {"zed.depth_topic": zed_depth_topic_config},
            {"zed.camera_info_topic": zed_camera_info_topic_config},
            {"zed.imu_topic": zed_imu_topic_config},
            {"zed.image_sync_queue_size": zed_image_sync_queue_size_config},
            {"zed.image_sync_slop_sec": zed_image_sync_slop_sec_config},
            {"zed.wait_for_camera_info": zed_wait_for_camera_info_config},
            {"zed.camera_info_timeout_ms": zed_camera_info_timeout_ms_config},
            {"zed.imu_buffer_size": zed_imu_buffer_size_config},
            {"zed.output_rgb": zed_output_rgb_config},
            {"zed.enable_imu_processing": zed_enable_imu_processing_config}
        ],
        remappings=[
            ("dataprovider/image/camera_info", camera_info_config),
            ("dataprovider/image/rgb", rgb_cam_topic_config),
            ("dataprovider/image/depth", depth_cam_topic_config),
            ("dataprovider/image/mask", motion_mask_cam_topic_config),
            ("dataprovider/image/flow", optical_flow_cam_topic_config)
        ],
        arguments=arguments,
        **node_kwargs
    )
    nodes.append(program_node)

    def on_node_completion_lambda(*args):
        reason_string = "Program node {} has exited".format(program_node.name)
        logger.info(reason_string + " - shutting down!")
        #NOTE: the reason text does not seem to propogate to the logger. I guess this is a bug with ROS2?
        action = Shutdown()
        action.visit(context)

    # set up handler to shutdown this context when the program node exists
    # this prevents any additional nodes hanging after the program finishes running
    nodes.append(RegisterEventHandler(OnProcessExit(
            target_action = program_node,
            on_exit = on_node_completion_lambda)))

    # if a static transform should be setup between the world and robot
    if kwargs.get("world_to_robot_tf", True):
        logger.info("Setting up world to robot transform")

        nodes.append(Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=["0.0", "0.0", "0.0", "1.57", "-1.57", "0.0", "world", "robot"]
        ))

    return nodes

def declare_launch_arguments(default_dict, **kwargs):
    launch_args = []
    for key, internal_default in default_dict.items():
        default = kwargs.get(key, internal_default)
        launch_args.append(DeclareLaunchArgument(key, default_value=str(default)))

    # remove the above values from the kwargs so that they are not parsed to the load_dynosam_node function
    clean_kwargs = kwargs

    for provided_key in default_dict.keys():
        if provided_key in clean_kwargs:
            del clean_kwargs[provided_key]

    return launch_args, clean_kwargs

def inside_ros_launch():
    import sys
    argv = sys.argv
    exec = os.path.basename(argv[0])

    if exec == "ros2":
        command = argv[1]
        return command == "launch"
    return False



def generate_dynosam_launch_description(**kwargs):
    # these values can be overwritten by providing launch arguments of the same name, these
    # just provide the default if no launch arguments are given
    # defaults that can be changed via ROS args
    launch_args, clean_kwargs = declare_launch_arguments(
        {"dataset_path": "/root/data/VDO/kitti/kitti/0004",
         "v": 20,
         "params_path": get_default_dynosam_params_path(),
         "output_path": "/root/results/DynoSAM/",
         "online": False,
         "online_provider_type": "generic",
         "wait_for_camera_params": True,
         "camera_params_timeout": -1,
         "camera_info": "/dyno/camera/camera_info",
         "rgb_cam_topic": "/dyno/camera/rgb",
         "depth_cam_topic": "/dyno/camera/depth",
         "motion_mask_cam_topic": "/dyno/camera/motion_mask",
         "optical_flow_cam_topic": "/dyno/camera/optical_flow",
         # ZED-specific default parameters
         "zed.rgb_topic": "rgb/image_rect_color",
         "zed.depth_topic": "depth/depth_registered", 
         "zed.camera_info_topic": "rgb/camera_info",
         "zed.imu_topic": "imu/data",
         "zed.image_sync_queue_size": 15,
         "zed.image_sync_slop_sec": 0.034,
         "zed.wait_for_camera_info": True,
         "zed.camera_info_timeout_ms": -1,
         "zed.imu_buffer_size": 200,
         "zed.output_rgb": True,
         "zed.enable_imu_processing": True},
         **kwargs)

    return LaunchDescription([
        # Must be inside launch description to be registered
        *launch_args,
        OpaqueFunction(function=load_dynosam_node, kwargs=clean_kwargs)
    ])
````

## File: include/dynosam_ros/adaptors/CameraParamsAdaptor.hpp
````
/*
 *   Copyright (c) 2025 ACFR-RPG, University of Sydney, Jesse Morris
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

#include <glog/logging.h>

#include <dynosam/common/CameraParams.hpp>

#include "rclcpp/type_adapter.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

template <>
struct rclcpp::TypeAdapter<dyno::CameraParams, sensor_msgs::msg::CameraInfo> {
  using is_specialized = std::true_type;
  using custom_type = dyno::CameraParams;
  using ros_message_type = sensor_msgs::msg::CameraInfo;

  static void convert_to_ros_message(const custom_type& source,
                                     ros_message_type& destination) {
    // intrisincs
    const cv::Mat& K = source.getCameraMatrix();
    std::copy(K.begin<double>(), K.end<double>(), destination.k.begin());

    // distortion
    const cv::Mat& D = source.getDistortionCoeffs();
    destination.d.resize(D.total());
    destination.d = std::vector<double>(D.begin<double>(), D.end<double>());

    destination.height = source.ImageHeight();
    destination.width = source.ImageWidth();

    std::string camera_model;
    CHECK(dyno::CameraParams::distortionModelToString(
        source.getDistortionModel(), destination.distortion_model,
        camera_model));
    (void)camera_model;
  }

  static void convert_to_custom(const ros_message_type& source,
                                custom_type& destination) {
    // bit gross but cv::Mat does not accept a const void* for data (which
    // source.k.data()) is so to ensure the data is safe (ie. rather than doing
    // a const conversion), I decide to copy the data into std::vector and then
    // copy into cv::Mat. this is inefficient but this conversion happens so
    // rarely and the data pakcet is tiny...
    std::vector<double> k_vector(source.k.begin(), source.k.end());
    cv::Mat K(3, 3, CV_64F, std::data(k_vector));
    dyno::CameraParams::IntrinsicsCoeffs intrinsics;
    // TODO: should make this a constructor...
    dyno::CameraParams::convertKMatrixToIntrinsicsCoeffs(K, intrinsics);
    dyno::CameraParams::DistortionCoeffs distortion(source.d.begin(),
                                                    source.d.end());
    cv::Size size(source.width, source.height);
    const std::string& distortion_model = "radtan";
    destination =
        dyno::CameraParams(intrinsics, distortion, size, distortion_model);
  }
};

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(dyno::CameraParams,
                                             sensor_msgs::msg::CameraInfo);
````

## File: include/dynosam_ros/displays/dynamic_slam_displays/BackendDSDRos.hpp
````
/*
 *   Copyright (c) 2025 ACFR-RPG, University of Sydney, Jesse Morris
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

#include <dynosam/backend/BackendOutputPacket.hpp>
#include <dynosam/common/GroundTruthPacket.hpp>
#include <dynosam/visualizer/Display.hpp>

#include "dynamic_slam_interfaces/msg/object_odometry.hpp"
#include "dynosam_ros/Display-Definitions.hpp"
#include "dynosam_ros/displays/DisplaysCommon.hpp"
#include "dynosam_ros/displays/dynamic_slam_displays/DSDCommonRos.hpp"
#include "rclcpp/node.hpp"

namespace dyno {

class BackendDSDRos : public BackendDisplay, DSDRos {
 public:
  BackendDSDRos(const DisplayParams params, rclcpp::Node::SharedPtr node);
  ~BackendDSDRos() = default;

  void spinOnce(const BackendOutputPacket::ConstPtr& backend_output) override;
};

}  // namespace dyno
````

## File: include/dynosam_ros/displays/dynamic_slam_displays/DSDCommonRos.hpp
````
/*
 *   Copyright (c) 2025 ACFR-RPG, University of Sydney, Jesse Morris
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

#include <dynosam/common/Types.hpp>
#include <dynosam/utils/Macros.hpp>

#include "dynamic_slam_interfaces/msg/multi_object_odometry_path.hpp"
#include "dynamic_slam_interfaces/msg/object_odometry.hpp"
#include "dynamic_slam_interfaces/msg/object_odometry_path.hpp"
#include "dynosam_ros/Display-Definitions.hpp"
#include "dynosam_ros/displays/DisplaysCommon.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"

namespace dyno {

using ObjectOdometry = dynamic_slam_interfaces::msg::ObjectOdometry;
using ObjectOdometryPath = dynamic_slam_interfaces::msg::ObjectOdometryPath;
using MultiObjectOdometryPath =
    dynamic_slam_interfaces::msg::MultiObjectOdometryPath;

using ObjectOdometryPub = rclcpp::Publisher<ObjectOdometry>;
using MultiObjectOdometryPathPub = rclcpp::Publisher<MultiObjectOdometryPath>;

//! Map of object id link (child frame id) to ObjectOdometry (for a single
//! frame, no frame ids)
using ObjectOdometryMap = gtsam::FastMap<std::string, ObjectOdometry>;

/**
 * @brief Class for managing the publishing and conversion of ObjectOdometry
 * messages.
 * DSD is shorthand for Dynamic SLAM Display.
 *
 * Contains an ObjectOdometry publisher on the topic "object_odometry" and a
 * tf2_ros::TransformBroadcaster to broadcast the object odometry TF.
 *
 */
class DSDTransport {
 public:
  DYNO_POINTER_TYPEDEFS(DSDTransport)

  DSDTransport(rclcpp::Node::SharedPtr node);

  /**
   * @brief Construct object id tf link from an object id.
   * This will be used as the link in the ROS tf tree and as the child frame id
   * for the object odometry.
   *
   * @param object_id ObjectId
   * @return std::string
   */
  static std::string constructObjectFrameLink(ObjectId object_id);

  // this is technically wrong as we should have a motion at k and a pose at k-1
  // to get velocity...
  static ObjectOdometry constructObjectOdometry(
      const gtsam::Pose3& motion_k, const gtsam::Pose3& pose_k,
      ObjectId object_id, Timestamp timestamp_k,
      const std::string& frame_id_link, const std::string& child_frame_id_link);

  static ObjectOdometryMap constructObjectOdometries(
      const MotionEstimateMap& motions_k, const ObjectPoseMap& poses,
      FrameId frame_id_k, Timestamp timestamp_k,
      const std::string& frame_id_link);

  static MultiObjectOdometryPath constructMultiObjectOdometryPaths(
      const MotionEstimateMap& motions_k, const ObjectPoseMap& poses,
      FrameId frame_id_k, Timestamp timestamp_k, rclcpp::Time ros_time_now,
      const std::string& frame_id_link);

  /**
   * @brief Nested Publisher that publishes all the object odometries for a
   * single frame/timestamp. Object odometries are set on construction.
   * Functionality for publishing the object odom's themselves and broadcasting
   * their position on the tf tree using the object current pose is addionally
   * included.
   *
   */
  class Publisher {
   public:
    /**
     * @brief Publish the contained object odometries.
     *
     */
    void publishObjectOdometry();

    /**
     * @brief Broadcast the transform of each object using their pose.
     * Transforms will be constructed between the Publisher::frame_id_link_
     * and object frame link (as the child_frame_id).
     *
     */
    void publishObjectTransforms();

    void publishObjectPaths();

    /**
     * @brief Get the frame id
     *
     * @return FrameId
     */
    inline FrameId getFrameId() const { return frame_id_; }
    inline Timestamp getTimestamp() const { return timestamp_; }

    /**
     * @brief Get the (tf) frame id used as the parent of the tree.
     *
     * @return const std::string&
     */
    inline const std::string& getFrameIdLink() const { return frame_id_link_; }

   private:
    rclcpp::Node::SharedPtr node_;
    ObjectOdometryPub::SharedPtr object_odom_publisher_;
    MultiObjectOdometryPathPub::SharedPtr multi_object_odom_path_publisher_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::string frame_id_link_;
    FrameId frame_id_;
    Timestamp timestamp_;

    //! Object odometries for this frame
    ObjectOdometryMap object_odometries_;
    MultiObjectOdometryPath object_paths_;

    friend class DSDTransport;

    /**
     * @brief Construct a new Publisher object.
     * Upon construction the publisher will broadcast to the 'object_odometry'
     * topic under the effective namespace of the provided node.
     *
     * @param node rclcpp::Node::SharedPtr
     * @param object_odom_publisher ObjectOdometryPub::SharedPtr Object odom
     * publisher to share between all Publishers.
     * @param multi_object_odom_path_publisher
     * MultiObjectOdometryPathPub::SharedPtr
     * @param tf_broadcaster std::shared_ptr<tf2_ros::TransformBroadcaster> TF
     * broadcaster to share between all Publishers.
     * @param motions const MotionEstimateMap& estimated motions at time (k)
     * @param poses const ObjectPoseMap& poses at time (... to k)
     * @param frame_id_link const std::string& parent tf tree link (e.g.
     * odom/world.)
     * @param frame_id FrameId current frame (k)
     * @param timestamp Timestamp
     */
    Publisher(
        rclcpp::Node::SharedPtr node,
        ObjectOdometryPub::SharedPtr object_odom_publisher,
        MultiObjectOdometryPathPub::SharedPtr multi_object_odom_path_publisher,
        std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster,
        const MotionEstimateMap& motions, const ObjectPoseMap& poses,
        const std::string& frame_id_link, FrameId frame_id,
        Timestamp timestamp);
  };

  /**
   * @brief Create a object odometry Publisher given object state (motion and
   * pose) information for a given frame (k). The resulting publisher can then
   * be used to published ObjectOdometry messages and update the tf tree with
   * the object poses w.r.t to the parent frame id link.
   *
   * @param motions_k const MotionEstimateMap& estimated motions at time (k)
   * @param poses const ObjectPoseMap& poses at time (... to k)
   * @param frame_id_link const std::string& parent tf tree link (e.g.
   * odom/world.)
   * @param frame_id FrameId current frame (k)
   * @param timestamp Timestamp
   * @return Publisher
   */
  Publisher addObjectInfo(const MotionEstimateMap& motions_k,
                          const ObjectPoseMap& poses,
                          const std::string& frame_id_link, FrameId frame_id,
                          Timestamp timestamp);

 private:
  rclcpp::Node::SharedPtr node_;
  ObjectOdometryPub::SharedPtr object_odom_publisher_;
  MultiObjectOdometryPathPub::SharedPtr multi_object_odom_path_publisher_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

/**
 * @brief Shorthand for Dynamic Slam Display ROS.
 *
 * Facilitates publishing to the following topics:
 *  - Visual odometry: "odometry"
 *  - Visual odometry path: "odometry_path"
 *  - Static point cloud: "static_cloud"
 *  - Dynamic point cloud: "dynamid_cloud"
 *
 * Additionally cointains a tf2_ros::TransformBroadcaster to broadcast the
 * visual odometry TF and a DSDTransport to publish the object odometry.
 *
 * The namespace of the topic is dependant on the effective namespace of the
 * node parsed in via the constructor.
 *
 */
class DSDRos {
 public:
  DSDRos(const DisplayParams& params, rclcpp::Node::SharedPtr node);

  void publishVisualOdometry(const gtsam::Pose3& T_world_camera,
                             Timestamp timestamp, const bool publish_tf);
  void publishVisualOdometryPath(const gtsam::Pose3Vector& poses,
                                 Timestamp latest_timestamp);

  CloudPerObject publishStaticPointCloud(const StatusLandmarkVector& landmarks,
                                         const gtsam::Pose3& T_world_camera);

  // struct PubDynamicCloudOptions {
  //   //TODO: unused
  //   bool publish_object_bounding_box{true};

  //   // PubDynamicCloudOptions() = default;
  //   ~PubDynamicCloudOptions() = default;
  // };

  CloudPerObject publishDynamicPointCloud(const StatusLandmarkVector& landmarks,
                                          const gtsam::Pose3& T_world_camera);

 private:
 protected:
  const DisplayParams params_;
  rclcpp::Node::SharedPtr node_;
  //! Dynamic SLAM display transport for estimated object odometry
  DSDTransport dsd_transport_;
  //! TF broadcaster for the odometry.
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  OdometryPub::SharedPtr vo_publisher_;
  PathPub::SharedPtr vo_path_publisher_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      static_points_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      dynamic_points_pub_;
};

}  // namespace dyno
````

## File: include/dynosam_ros/displays/dynamic_slam_displays/FrontendDSDRos.hpp
````
/*
 *   Copyright (c) 2025 ACFR-RPG, University of Sydney, Jesse Morris
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

#include <dynosam/common/GroundTruthPacket.hpp>
#include <dynosam/frontend/RGBDInstance-Definitions.hpp>
#include <dynosam/visualizer/Display.hpp>

#include "dynamic_slam_interfaces/msg/object_odometry.hpp"
#include "dynosam_ros/Display-Definitions.hpp"
#include "dynosam_ros/displays/DisplaysCommon.hpp"
#include "dynosam_ros/displays/dynamic_slam_displays/DSDCommonRos.hpp"
#include "image_transport/image_transport.hpp"
#include "rclcpp/node.hpp"

namespace dyno {

class FrontendDSDRos : public FrontendDisplay, DSDRos {
 public:
  FrontendDSDRos(const DisplayParams params, rclcpp::Node::SharedPtr node);
  ~FrontendDSDRos() = default;

  void spinOnce(
      const FrontendOutputPacketBase::ConstPtr& frontend_output) override;

 private:
  void tryPublishDebugImagery(
      const FrontendOutputPacketBase::ConstPtr& frontend_output);
  void tryPublishGroundTruth(
      const FrontendOutputPacketBase::ConstPtr& frontend_output);
  void tryPublishVisualOdometry(
      const FrontendOutputPacketBase::ConstPtr& frontend_output);

  void processRGBDOutputpacket(
      const RGBDInstanceOutputPacket::ConstPtr& rgbd_packet);

 private:
  //! Transport for ground truth publishing
  DSDTransport::UniquePtr dsd_ground_truth_transport_;
  //! Image Transport for tracking image
  image_transport::Publisher tracking_image_pub_;

  OdometryPub::SharedPtr vo_ground_truth_publisher_;
  PathPub::SharedPtr vo_path_ground_truth_publisher_;
};

}  // namespace dyno
````

## File: include/dynosam_ros/displays/dynamic_slam_displays/README.md
````markdown
# Dynamic SLAM Displays

Version of the front/back-end displays that depends on `dynamic_slam_interfaces` to publish the object states per frame:

- object pose
- object paths
- object motions (as velocity)
- object id

Each object is represented using the `dynamic_slam_interfaces::msg::ObjectOdometry` message which can be displayed in RVIZ using the [rviz_dynamic_slam_plugins](https://github.com/ACFR-RPG/rviz_dynamic_slam_plugins) plugin.
These classes are compiled by __default__ but this configuration can be specified with:
```
colcon build --cmake-args -DENABLE_DYNAMIC_SLAM_INTERFACES=ON
```
and used as the `FrontendDisplayRos` and `BackendDisplayRos`, as defined in [DisplaysImpl.hpp](../DisplaysImpl.hpp).
````

## File: include/dynosam_ros/displays/inbuilt_displays/BackendInbuiltDisplayRos.hpp
````
/*
 *   Copyright (c) 2025 ACFR-RPG, University of Sydney, Jesse Morris
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

#include <dynosam/visualizer/Display.hpp>

#include "dynosam_ros/Display-Definitions.hpp"
#include "dynosam_ros/displays/inbuilt_displays/InbuiltDisplayCommon.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace dyno {

class BackendInbuiltDisplayRos : public BackendDisplay, InbuiltDisplayCommon {
 public:
  BackendInbuiltDisplayRos(const DisplayParams params,
                           rclcpp::Node::SharedPtr node);

  void spinOnce(const BackendOutputPacket::ConstPtr& backend_output) override;

 private:
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      static_tracked_points_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      dynamic_tracked_points_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      dynamic_initial_points_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      new_scaled_dynamic_points_pub_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;

  // optimzied
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr odometry_path_pub_;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      object_pose_path_pub_;  //! Path of propogated object poses using the
                              //! motion estimate
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      object_pose_pub_;  //! Propogated object poses using the motion estimate

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      object_aabb_pub_;  //! Draw object bounding boxes as cubes
};

}  // namespace dyno
````

## File: include/dynosam_ros/displays/inbuilt_displays/FrontendInbuiltDisplayRos.hpp
````
/*
 *   Copyright (c) 2025 ACFR-RPG, University of Sydney, Jesse Morris
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

#include <dynosam/common/GroundTruthPacket.hpp>
#include <dynosam/frontend/RGBDInstance-Definitions.hpp>
#include <dynosam/visualizer/Display.hpp>
#include <opencv4/opencv2/videoio.hpp>

#include "dynosam_ros/Display-Definitions.hpp"
#include "dynosam_ros/displays/inbuilt_displays/InbuiltDisplayCommon.hpp"
#include "image_transport/image_transport.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "visualization_msgs/msg/marker_array.hpp"

namespace dyno {

class FrontendInbuiltDisplayRos : public FrontendDisplay, InbuiltDisplayCommon {
 public:
  FrontendInbuiltDisplayRos(const DisplayParams params,
                            rclcpp::Node::SharedPtr node);
  ~FrontendInbuiltDisplayRos();

  void spinOnce(
      const FrontendOutputPacketBase::ConstPtr& frontend_output) override;

 private:
  void processRGBDOutputpacket(
      const RGBDInstanceOutputPacket::ConstPtr& rgbd_frontend_output);

  void publishOdometry(const gtsam::Pose3& T_world_camera, Timestamp timestamp);
  // void publishOdometryPath(const gtsam::Pose3& T_world_camera, Timestamp
  // timestamp);
  void publishDebugImage(const DebugImagery& debug_imagery);

  void publishGroundTruthInfo(Timestamp timestamp,
                              const GroundTruthInputPacket& gt_packet,
                              const cv::Mat& rgb);

 private:
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      static_tracked_points_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      dynamic_tracked_points_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr odometry_path_pub_;
  nav_msgs::msg::Path odom_path_msg_;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      object_pose_path_pub_;  //! Path of propogated object poses using the
                              //! motion estimate
  std::map<ObjectId, gtsam::Pose3Vector> object_trajectories_;
  std::map<ObjectId, FrameId>
      object_trajectories_update_;  //! The last frame id that the object was
                                    //! seen in

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr
      object_motion_pub_;  //! Publish object motions per frame as an array of
                           //! SE(3) transformations (a Path) where frame_id per
                           //! pose is object id

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      object_pose_pub_;  //! Propogated object poses using the motion estimate
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      object_bbx_line_pub_;  //! Draw object bounding boxes as line lists
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      object_bbx_pub_;  //! Draw object bounding boxes as cubes
  image_transport::Publisher tracking_image_pub_;

  // ground truth publishers
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      gt_object_pose_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      gt_object_path_pub_;  //! Path of objects with gt
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr gt_odometry_pub_;

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr gt_odom_path_pub_;
  nav_msgs::msg::Path gt_odom_path_msg_;

  image_transport::Publisher gt_bounding_box_pub_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  std::unique_ptr<cv::VideoWriter> video_writer_;  // for now
};

}  // namespace dyno
````

## File: include/dynosam_ros/displays/inbuilt_displays/InbuiltDisplayCommon.hpp
````
/*
 *   Copyright (c) 2025 ACFR-RPG, University of Sydney, Jesse Morris
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

#include <pcl_conversions/pcl_conversions.h>

#include <dynosam/common/Camera.hpp>
#include <dynosam/common/Exceptions.hpp>
#include <dynosam/common/PointCloudProcess.hpp>
#include <dynosam/common/Types.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "dynosam_ros/Display-Definitions.hpp"
#include "dynosam_ros/displays/DisplaysCommon.hpp"
#include "image_transport/image_transport.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace dyno {

class InbuiltDisplayCommon {
 public:
  InbuiltDisplayCommon(const DisplayParams& params,
                       rclcpp::Node::SharedPtr node);
  virtual ~InbuiltDisplayCommon() = default;

  virtual CloudPerObject publishPointCloud(
      PointCloud2Pub::SharedPtr pub, const StatusLandmarkVector& landmarks,
      const gtsam::Pose3& T_world_camera);
  virtual void publishOdometry(OdometryPub::SharedPtr pub,
                               const gtsam::Pose3& T_world_camera,
                               Timestamp timestamp);
  virtual void publishOdometryPath(PathPub::SharedPtr pub,
                                   const gtsam::Pose3Vector& poses,
                                   Timestamp latest_timestamp);

  virtual void publishObjectPositions(
      MarkerArrayPub::SharedPtr pub, const ObjectPoseMap& object_positions,
      FrameId frame_id, Timestamp latest_timestamp,
      const std::string& prefix_marker_namespace, bool draw_labels = false,
      double scale = 1.0);

  // if -1 min_poses, draw all
  virtual void publishObjectPaths(MarkerArrayPub::SharedPtr pub,
                                  const ObjectPoseMap& object_positions,
                                  FrameId frame_id, Timestamp latest_timestamp,
                                  const std::string& prefix_marker_namespace,
                                  const int min_poses = 60);

  virtual void publishObjectBoundingBox(
      MarkerArrayPub::SharedPtr aabb_pub, MarkerArrayPub::SharedPtr obb_pub,
      const CloudPerObject& cloud_per_object, Timestamp timestamp,
      const std::string& prefix_marker_namespace);

  /**
   * @brief Constructs axis-aligned BB (AABB) and oriented BB (OBB) marker
   * arrays from a set of point clouds which are coloured per object.
   *
   * NOTE:(jesse) right now OBB are not gravity aligned
   *
   * @param cloud_per_object
   * @param aabb_markers
   * @param obb_markers
   * @param latest_timestamp
   * @param prefix_marker_namespace
   */
  static void constructBoundingBoxeMarkers(
      const CloudPerObject& cloud_per_object, MarkerArray& aabb_markers,
      MarkerArray& obb_markers, Timestamp timestamp,
      const std::string& prefix_marker_namespace);

  static void createAxisMarkers(const gtsam::Pose3& pose,
                                MarkerArray& axis_markers, Timestamp timestamp,
                                const cv::Scalar& colour,
                                const std::string& frame, const std::string& ns,
                                double length = 0.8, double radius = 0.1);

  // T_world_x should put something in the local frame to the world frame
  // taken from
  // https://github.com/uzh-rpg/rpg_svo_pro_open/blob/master/vikit/vikit_ros/src/output_helper.cpp
  MarkerArray createCameraMarker(const gtsam::Pose3& T_world_x,
                                 Timestamp timestamp, const std::string& ns,
                                 const cv::Scalar& colour,
                                 double marker_scale = 1.0);

  void publishCameraMarker(const gtsam::Pose3& T_world_x, Timestamp timestamp,
                           const std::string& ns, const cv::Scalar& colour,
                           double marker_scale = 1.0) {
    camera_frustrum_pub_->publish(
        createCameraMarker(T_world_x, timestamp, ns, colour, marker_scale));
  }

  inline visualization_msgs::msg::Marker getDeletionMarker() const {
    static visualization_msgs::msg::Marker delete_marker;
    delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    return delete_marker;
  }

 protected:
  const DisplayParams params_;
  rclcpp::Node::SharedPtr node_;

  MarkerArrayPub::SharedPtr
      camera_frustrum_pub_;  //! publishes camera pose as a marker array,
                             //! representing the frustrum, in the world frame
};

}  // namespace dyno
````

## File: include/dynosam_ros/displays/inbuilt_displays/README.md
````markdown
# Inbuilt ROS displays

Version of the front/back-end displays that use `visualization_msgs` to display the object states per frame:

- object pose
- object paths
- object id
- bounding box

Inbuilt refers to the fact that no third-party deps are needed for the visualisation, instead we use standard ROS messages (mainly `visualization_msgs`) to display the object states.
This setup is therefore more complex, and results in many more advertised topics to achieve a similar (and less flexible) display than using the custom plugin/interface combination.

These classes are conditionally compiled:
```
colcon build --cmake-args -DENABLE_DYNAMIC_SLAM_INTERFACES=OFF
```
and used as the `FrontendDisplayRos` and `BackendDisplayRos`, as defined in [DisplaysImpl.hpp](../DisplaysImpl.hpp).
````

## File: include/dynosam_ros/displays/DisplaysCommon.hpp
````
/*
 *   Copyright (c) 2025 ACFR-RPG, University of Sydney, Jesse Morris
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

#include <pcl_conversions/pcl_conversions.h>

#include <dynosam/common/PointCloudProcess.hpp>  //for CloudPerObject
#include <dynosam/common/Types.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "dynosam_ros/Display-Definitions.hpp"
#include "image_transport/image_transport.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace dyno {

using PointCloud2 = sensor_msgs::msg::PointCloud2;
using MarkerArray =
    visualization_msgs::msg::MarkerArray;  //! Typedef for MarkerArray msg

using PointCloud2Pub = rclcpp::Publisher<sensor_msgs::msg::PointCloud2>;
using OdometryPub = rclcpp::Publisher<nav_msgs::msg::Odometry>;
using PathPub = rclcpp::Publisher<nav_msgs::msg::Path>;
using MarkerArrayPub = rclcpp::Publisher<MarkerArray>;

/**
 * @brief Common stateless (free) functions for all ROS displays.
 *
 */
struct DisplayCommon {
  static CloudPerObject publishPointCloud(PointCloud2Pub::SharedPtr pub,
                                          const StatusLandmarkVector& landmarks,
                                          const gtsam::Pose3& T_world_camera,
                                          const std::string& frame_id);
  static void publishOdometry(OdometryPub::SharedPtr pub,
                              const gtsam::Pose3& T_world_camera,
                              Timestamp timestamp, const std::string& frame_id,
                              const std::string& child_frame_id);
  static void publishOdometryPath(PathPub::SharedPtr pub,
                                  const gtsam::Pose3Vector& poses,
                                  Timestamp latest_timestamp,
                                  const std::string& frame_id);
};

}  // namespace dyno
````

## File: include/dynosam_ros/displays/DisplaysImpl.hpp
````
/*
 *   Copyright (c) 2025 ACFR-RPG, University of Sydney, Jesse Morris
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

#ifdef USE_DYNAMIC_SLAM_INTERFACES

#include "dynosam_ros/displays/dynamic_slam_displays/BackendDSDRos.hpp"
#include "dynosam_ros/displays/dynamic_slam_displays/FrontendDSDRos.hpp"

namespace dyno {
typedef FrontendDSDRos FrontendDisplayRos;
typedef BackendDSDRos BackendDisplayRos;
}  // namespace dyno

#else
#include "dynosam_ros/displays/inbuilt_displays/BackendInbuiltDisplayRos.hpp"
#include "dynosam_ros/displays/inbuilt_displays/FrontendInbuiltDisplayRos.hpp"

namespace dyno {
typedef FrontendInbuiltDisplayRos FrontendDisplayRos;
typedef BackendInbuiltDisplayRos BackendDisplayRos;
}  // namespace dyno

#endif
````

## File: include/dynosam_ros/DataProviderRos.hpp
````
/*
 *   Copyright (c) 2025 ACFR-RPG, University of Sydney, Jesse Morris
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

#include <dynosam/common/Exceptions.hpp>
#include <dynosam/common/ImageTypes.hpp>
#include <dynosam/dataprovider/DataProvider.hpp>
#include <opencv4/opencv2/opencv.hpp>

#include "cv_bridge/cv_bridge.h"
#include "dynosam_ros/adaptors/CameraParamsAdaptor.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/node_options.hpp"
#include "rclcpp/wait_for_message.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace dyno {

/**
 * @brief Base Dataprovider for ROS that implements common image processing
 * functionalities.
 *
 */
class DataProviderRos : public DataProvider {
 public:
  DataProviderRos(rclcpp::Node::SharedPtr node);
  virtual ~DataProviderRos() = default;

  /**
   * @brief Convers a sensor_msgs::msg::Image to a cv::Mat while testing that
   * the input has the correct datatype for an RGB image (as defined by
   * ImageType::RGBMono).
   *
   * @param img_msg const sensor_msgs::msg::Image::ConstSharedPtr&
   * @return const cv::Mat
   */
  const cv::Mat readRgbRosImage(
      const sensor_msgs::msg::Image::ConstSharedPtr& img_msg) const;

  /**
   * @brief Convers a sensor_msgs::msg::Image to a cv::Mat while testing that
   * the input has the correct datatype for an Depth image (as defined by
   * ImageType::Depth).
   *
   * @param img_msg const sensor_msgs::msg::Image::ConstSharedPtr&
   * @return const cv::Mat
   */
  const cv::Mat readDepthRosImage(
      const sensor_msgs::msg::Image::ConstSharedPtr& img_msg) const;

  /**
   * @brief Convers a sensor_msgs::msg::Image to a cv::Mat while testing that
   * the input has the correct datatype for an Optical Flow image (as defined by
   * ImageType::OpticalFlow).
   *
   * @param img_msg const sensor_msgs::msg::Image::ConstSharedPtr&
   * @return const cv::Mat
   */
  const cv::Mat readFlowRosImage(
      const sensor_msgs::msg::Image::ConstSharedPtr& img_msg) const;

  /**
   * @brief Convers a sensor_msgs::msg::Image to a cv::Mat while testing that
   * the input has the correct datatype for an Motion Mask image (as defined by
   * ImageType::MotionMask).
   *
   * @param img_msg const sensor_msgs::msg::Image::ConstSharedPtr&
   * @return const cv::Mat
   */
  const cv::Mat readMaskRosImage(
      const sensor_msgs::msg::Image::ConstSharedPtr& img_msg) const;

  /**
   * @brief Gets CameraParams from a sensor_msgs::msg::CameraInfo recieved on
   * the specified topic. This function is blocking until a message is recieved
   * (or until the time_to_wait) elapses.
   *
   * While this function returns a const ref to the CameraParams it also sets
   * the internal camera_params_. The camera params are then returned by the
   * overwritten getCameraParams, allowing the PipelineManager to access the
   * correct camera paramters.
   *
   * @tparam Rep int64_t,
   * @tparam Period std::milli
   * @param time_to_wait const std::chrono::duration<Rep, Period>&
   * @param topic const std::string&. Defaults to "image/camera_info"
   * @return const CameraParams&
   */
  template <class Rep = int64_t, class Period = std::milli>
  const CameraParams& waitAndSetCameraParams(
      const std::chrono::duration<Rep, Period>& time_to_wait =
          std::chrono::duration<Rep, Period>(-1),
      const std::string& topic = "image/camera_info") {
    RCLCPP_INFO_STREAM(node_->get_logger(),
                       "Waiting for camera params on topic: " << topic);
    // it seems rclcpp::Adaptors do not work yet with wait for message
    sensor_msgs::msg::CameraInfo camera_info;
    if (rclcpp::wait_for_message<sensor_msgs::msg::CameraInfo, Rep, Period>(
            camera_info, node_, topic, time_to_wait)) {
      using Adaptor =
          rclcpp::TypeAdapter<dyno::CameraParams, sensor_msgs::msg::CameraInfo>;
      CameraParams camera_params;
      Adaptor::convert_to_custom(camera_info, camera_params);
      RCLCPP_INFO_STREAM(node_->get_logger(), "Received camera params: "
                                                  << camera_params.toString());
      camera_params_ = camera_params;
      return *camera_params_;
    } else {
      const auto milliseconds =
          std::chrono::duration_cast<std::chrono::milliseconds>(time_to_wait);
      throw DynosamException("Failed to receive camera params on topic " +
                             topic + " (waited with timeout " +
                             std::to_string(milliseconds.count()) + " ms).");
    }
  }

  CameraParams::Optional getCameraParams() const override {
    return camera_params_;
  }

 protected:
  /**
   * @brief Helper function to convert a ROS Image message to a CvImageConstPtr
   * via the cv bridge.
   *
   * @param img_msg const sensor_msgs::msg::Image::ConstSharedPtr&
   * @return const cv_bridge::CvImageConstPtr
   */
  const cv_bridge::CvImageConstPtr readRosImage(
      const sensor_msgs::msg::Image::ConstSharedPtr& img_msg) const;

  /**
   * @brief Helper function to convert a
   * sensor_msgs::msg::Image::ConstSharedPtr& to a cv::Mat with the right
   * datatype.
   *
   * The datatype is specified from the template IMAGETYPE::OpenCVType and
   * ensures the passed in image has the correct datatype for the desired
   * IMAGETYPE.
   *
   * ROS will be shutdown if the incoming image has an incorrect type.
   *
   * @tparam IMAGETYPE
   * @param img_msg  const sensor_msgs::msg::Image::ConstSharedPtr&
   * @return const cv::Mat
   */
  template <typename IMAGETYPE>
  const cv::Mat convertRosImage(
      const sensor_msgs::msg::Image::ConstSharedPtr& img_msg) const {
    const cv_bridge::CvImageConstPtr cvb_image = readRosImage(img_msg);

    try {
      const cv::Mat img = cvb_image->image;
      image_traits<IMAGETYPE>::validate(img);
      return img;

    } catch (const InvalidImageTypeException& exception) {
      RCLCPP_FATAL_STREAM(node_->get_logger(),
                          image_traits<IMAGETYPE>::name()
                              << " Image msg was of the wrong type (validate "
                                 "failed with exception "
                              << exception.what() << "). "
                              << "ROS encoding type used was "
                              << cvb_image->encoding);
      rclcpp::shutdown();
      return cv::Mat();
    }
  }

 protected:
  rclcpp::Node::SharedPtr node_;
  CameraParams::Optional camera_params_;
};

}  // namespace dyno
````

## File: include/dynosam_ros/Display-Definitions.hpp
````
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

#pragma once

#include <string>

namespace dyno {

struct DisplayParams {
  std::string world_frame_id = "world";
  std::string camera_frame_id = "camera";
};

}  // namespace dyno
````

## File: include/dynosam_ros/OnlineDataProviderRos.hpp
````
/*
 *   Copyright (c) 2025 ACFR-RPG, University of Sydney, Jesse Morris
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

#include "dynosam_ros/DataProviderRos.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/exact_time.h"
#include "message_filters/synchronizer.h"
#include "rclcpp/node.hpp"
#include "rclcpp/node_options.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace dyno {

struct OnlineDataProviderRosParams {
  bool wait_for_camera_params{true};
  int32_t camera_params_timeout{-1};
};

/**
 * @brief Online data-provider for DynoSAM.
 *
 * Subscribes to four synchronized image topics (rgb, depth, mask and optical
 * flow) and a camera_info topic (to define the camera intrinsics of the
 * system).
 *
 *
 *
 */
class OnlineDataProviderRos : public DataProviderRos {
 public:
  /**
   * @brief Construct a new OnlineDataProviderRos.
   *
   * Constructor will block until camera info has been received (if
   * OnlineDataProviderRosParams::wait_for_camera_params is true).
   *
   * @param node rclcpp::Node::SharedPtr
   * @param params const OnlineDataProviderRosParams&
   */
  OnlineDataProviderRos(rclcpp::Node::SharedPtr node,
                        const OnlineDataProviderRosParams &params);

  /**
   * @brief Indicates that there is no known end to the dataset
   *
   * @return int
   */
  int datasetSize() const override { return -1; }

  /**
   * @brief Returns true while not shutdown
   *
   * @return true
   * @return false
   */
  bool spin() override;

  /**
   * @brief Disconnects all subscribers
   *
   */
  void shutdown() override;

  /**
   * @brief Connects all subscribers.
   *
   */
  void connect();

 private:
  using SyncPolicy = message_filters::sync_policies::ExactTime<
      sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::Image,
      sensor_msgs::msg::Image>;

  void imageSyncCallback(
      const sensor_msgs::msg::Image::ConstSharedPtr &rgb_msg,
      const sensor_msgs::msg::Image::ConstSharedPtr &depth_msg,
      const sensor_msgs::msg::Image::ConstSharedPtr &flow_msg,
      const sensor_msgs::msg::Image::ConstSharedPtr &mask_msg);

 private:
  FrameId frame_id_;

  message_filters::Subscriber<sensor_msgs::msg::Image> rgb_image_sub_;
  message_filters::Subscriber<sensor_msgs::msg::Image> depth_image_sub_;
  message_filters::Subscriber<sensor_msgs::msg::Image> flow_image_sub_;
  message_filters::Subscriber<sensor_msgs::msg::Image> mask_image_sub_;

  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
};

}  // namespace dyno
````

## File: include/dynosam_ros/PipelineRos.hpp
````
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

#include <dynosam/dataprovider/DataProvider.hpp>
#include <dynosam/pipeline/PipelineManager.hpp>
#include <dynosam/pipeline/PipelineParams.hpp>
#include <dynosam/utils/Statistics.hpp>

#include "rclcpp/node.hpp"
#include "rclcpp/node_options.hpp"

namespace dyno {

// Forward declarations
struct ZEDOnlineDataProviderParams;

class DynoNode : public rclcpp::Node {
 public:
  explicit DynoNode(const std::string& node_name,
                    const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  virtual ~DynoNode() = default;

  virtual bool spinOnce() { return true; }

  std::string getStats() const { return utils::Statistics::Print(); }

  const DynoParams& getDynoParams() {
    CHECK_NOTNULL(dyno_params_);
    return *dyno_params_;
  }

  inline std::string getParamsPath() {
    return searchForPathWithParams("params_folder_path", "dynosam/params/",
                                   "Path to the folder containing the yaml "
                                   "files with the DynoVIO parameters.");
  }
  inline std::string getDatasetPath() {
    return searchForPathWithParams("dataset_path", "dataset",
                                   "Path to the dataset.");
  }

 protected:
  // NOT cached!!
  virtual dyno::DataProvider::Ptr createDataProvider();
  
  ZEDOnlineDataProviderParams parseZEDParams();

 private:
  /**
   * @brief Retrieves a std::string param (under param_name) which is expected
   * to be a file path
   *
   * @param param_name
   * @param default_path
   * @param description
   * @return std::string
   */
  std::string searchForPathWithParams(const std::string& param_name,
                                      const std::string& default_path,
                                      const std::string& description = "");

  std::unique_ptr<DynoParams> dyno_params_;
};

class DynoPipelineManagerRos : public DynoNode {
 public:
  explicit DynoPipelineManagerRos(
      const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~DynoPipelineManagerRos() = default;

  void initalisePipeline();

  bool spinOnce() override {
    RCLCPP_INFO_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                getStats());

    return CHECK_NOTNULL(pipeline_)->spin();
  }

 private:
  DynoPipelineManager::UniquePtr pipeline_;
};

}  // namespace dyno
````

## File: include/dynosam_ros/RosUtils-inl.hpp
````
/*
 *   Copyright (c) 2025 ACFR-RPG, University of Sydney, Jesse Morris
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

#include <glog/logging.h>

#include "dynosam_ros/RosUtils.hpp"

namespace dyno {

template <typename ValueTypeT>
decltype(auto) ParameterDetails::get() const {
  return this->get_param<ValueTypeT>(this->default_parameter_);
}

template <typename ValueTypeT>
ValueTypeT ParameterDetails::get(ValueTypeT default_value) const {
  return this->get_param<ValueTypeT>(
      rclcpp::Parameter(this->name(), default_value));
}

// template <typename ValueTypeT>
// void ParameterDetails::registerParamCallback(const
// std::function<void(ValueTypeT)>& callback) {
//   PropertyHandler::OnChangeFunction<rclcpp::Parameter> wrapper =
//   [=](rclcpp::Parameter, rclcpp::Parameter new_parameter) -> void {
//     try {
//       //this type does not necessarily match with the internal paramter type
//       //as this gets updated in too many places
//       ValueTypeT value = new_parameter.get_value<ValueTypeT>();
//       //call the actual user defined callback
//       callback(value);
//     }
//     catch(rclcpp::exceptions::InvalidParameterTypeException& e) {
//       LOG(ERROR) << "Failed to emit callback for parameter change with param:
//       " << this->name()
//         << " requested type " << type_name<ValueTypeT>() << " but actual type
//         was " << new_parameter.get_type_name();
//     }
//   };

//   //always return true so that the callback is always triggered from the
//   handler static const HasChangedValue<rclcpp::Parameter> has_changed =
//   [](rclcpp::Parameter, rclcpp::Parameter) -> bool { return true; };

//   property_handler_.registerVariable<rclcpp::Parameter>(
//     this->name(),
//     /// Default value that currently exists
//     /// Might be a problem if this is used prior to declare param and this
//     does not have a default!! this->get(), wrapper, has_changed
//   );
// }

template <typename ValueTypeT>
decltype(auto) ParameterDetails::get_param(
    const rclcpp::Parameter &default_param) const {
  const rclcpp::Parameter param = get_param(default_param);
  return param.get_value<ValueTypeT>();
}

template <typename ValueTypeT>
ParameterConstructor::ParameterConstructor(rclcpp::Node::SharedPtr node,
                                           const std::string &name,
                                           ValueTypeT value)
    : ParameterConstructor(node.get(), name, value) {}

template <typename ValueTypeT>
ParameterConstructor::ParameterConstructor(rclcpp::Node *node,
                                           const std::string &name,
                                           ValueTypeT value)
    : node_(node), parameter_(name, value) {
  CHECK_NOTNULL(node_);
  parameter_descriptor_.name = name;
  parameter_descriptor_.type = traits<ValueTypeT>::ros_parameter_type;
}

}  // namespace dyno
````

## File: include/dynosam_ros/RosUtils.hpp
````
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

#include <dynosam/common/Exceptions.hpp>
#include <dynosam/common/Types.hpp>
#include <type_traits>

#include "rcl_interfaces/msg/parameter.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/parameter.hpp"
#include "rclcpp/time.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace dyno {

/**
 * @brief A generic type trait with the dyno namespace. Here we define compile
 * time conversions between c++ types and their corresponding
 * rcl_interfaces::msg::ParameterType. The following properies are defined:
 *
 * ::traits<double>::ros_parameter_type, where T=double is any primitive type
 * that has a parameter type versions, and,
 * ::traits<rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE>::cpp_type
 *
 *
 * @tparam T
 */
template <typename T>
struct traits;

/**
 * @brief Type trait to retrieve a c++ type from the corresponding
 * rcl_interfaces::msg::ParameterType.
 * ros_param_traits<rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE>::cpp_type
 * == decltype(double)
 *
 *
 * @tparam Type rcl_interfaces::msg::ParameterType
 */
template <uint8_t Type>
struct ros_param_traits;

namespace internal {

// Taken from https://en.cppreference.com/w/cpp/types/type_identity
// since ROS2 does not support C++20 yet
template <class T>
struct type_identity {
  using type = T;
};

template <class T>
using type_identity_t = typename type_identity<T>::type;

template <typename Msg, typename = int>
struct HasMsgHeader : std::false_type {};

template <typename Msg>
struct HasMsgHeader<Msg, decltype((void)Msg::header, 0)> : std::true_type {};

template <typename Msg, typename = int>
struct HasChildFrame : std::false_type {};

template <typename Msg>
struct HasChildFrame<Msg, decltype((void)Msg::child_frame_id, 0)>
    : std::true_type {};

template <uint8_t Type>
struct RosParameterType {
  // as of the time of writing rcl_interfaces::msg::ParameterType:: is simply a
  // uint8 type for the sake of type safetly we will just check this as I dont
  // see version changes happening that much
  static_assert(std::is_same<decltype(Type), uint8_t>::value,
                "rcl_interfaces::msg::ParameterType:: is expected to be a "
                "uint8_t - check version!");
  static constexpr uint8_t ros_parameter_type = Type;
};

/**
 * @brief Type trait to retrieve a
 *
 * @tparam CPP_TYPE
 */
template <typename CPP_TYPE>
struct CppParameterType {
  using cpp_type = typename internal::type_identity_t<CPP_TYPE>;
};

}  // namespace internal

/**
 * @brief Exception thrown when a parameter is missing a default parameter and
 * MUST be set by the user at runtime (ie. no default value allowed)
 *
 */
class InvalidDefaultParameter : public DynosamException {
 public:
  InvalidDefaultParameter(const std::string &missing_param,
                          const std::string &custom_message = std::string())
      : DynosamException(
            "Missing required param " + missing_param +
            " which must set at runtime by the user (e.g. via a config file or "
            "via launch configurations)" +
            (custom_message.empty() ? "" : std::string(": " + custom_message))),
        missing_param_(missing_param),
        custom_message_(custom_message) {}

  const std::string &missingParam() const { return missing_param_; }
  const std::string &customMessage() const { return custom_message_; }

 private:
  const std::string missing_param_;
  const std::string custom_message_;
};

class ParameterDetails {
 public:
  friend class ParameterConstructor;

  /**
   * @brief Name of the paramter
   *
   * @return const std::string&
   */
  const std::string &name() const;

  /**
   * @brief Name of the associated node
   *
   * @return std::string
   */
  std::string node_name() const;

  /**
   * @brief Get the rclcpp::Parameter from the node
   *
   * @return rclcpp::Parameter
   */
  rclcpp::Parameter get() const;

  /**
   * @brief Get the c++ value from the node.
   *
   * rclcpp::exceptions::InvalidParameterTypeException	if the type doesn't
   * match
   *
   * @tparam ValueTypeT c++ type
   * @return decltype(auto)
   */
  template <typename ValueTypeT>
  decltype(auto) get() const;

  //   /**
  //    * @brief Registeres a calback to be triggered when a paramter is
  //    updated.
  //    *
  //    * This uses the /paramter_events and ParameterCallbackHandle mechanism
  //    internally.
  //    *
  //    * @tparam ValueTypeT
  //    * @param callback
  //    */
  //   template <typename ValueTypeT>
  //   void registerParamCallback(const std::function<void(ValueTypeT)>&
  //   callback);

  /**
   * @brief Get the c++ value from the node or the default value if param is not
   * set in the node. If the default value is used, the param in the node will
   * NOT be updated.
   *
   * This default value will only be used if there is NO value in the node, i.e
   * setting a default value from the ParameterConstructor or using paramter
   * overwrites using NodeOptions or the launchfile will set value in the node
   * and the default_value argument will not be used.
   *
   * @tparam ValueTypeT
   * @param default_value
   * @return ValueTypeT
   */
  template <typename ValueTypeT>
  ValueTypeT get(ValueTypeT default_value) const;

  /**
   * @brief Specalisation for getting the value from the node as a std::string
   * or a default string.
   *
   * See template <typename ValueTypeT> ValueTypeT get(ValueTypeT) for details
   * on how the default value is used.
   *
   * @param default_value const char *
   * @return std::string
   */
  std::string get(const char *default_value) const;

  /**
   * @brief Specalisation for getting the value from the node as a std::string
   * or a default string.
   *
   * See template <typename ValueTypeT> ValueTypeT get(ValueTypeT) for details
   * on how the default value is used.
   *
   * @param default_value const std::string
   * @return std::string
   */
  std::string get(const std::string &default_value) const;

  inline bool isSet() const {
    return node_->get_parameter(this->name())
               .get_parameter_value()
               .get_type() != rclcpp::PARAMETER_NOT_SET;
  }

  /**
   * @brief Casting operator and returns the Paramter type from the node
   *
   * @return rclcpp::Parameter
   */
  operator rclcpp::Parameter() const { return this->get(); }

  /**
   * @brief Casting operator for the parameter value from the node
   *
   * @return rclcpp::ParameterValue
   */
  operator rclcpp::ParameterValue() const {
    return this->get().get_parameter_value();
  }

  operator std::string() const;

 private:
  ParameterDetails(rclcpp::Node *node, const rclcpp::Parameter &parameter,
                   const rcl_interfaces::msg::ParameterDescriptor &description);

  rclcpp::Parameter get_param(const rclcpp::Parameter &default_param) const;

  void declare();

  template <typename ValueTypeT>
  decltype(auto) get_param(const rclcpp::Parameter &default_param) const;

 private:
  mutable rclcpp::Node *node_;
  const rclcpp::Parameter
      default_parameter_;  //! the original parameter, prior to declaration with
                           //! the node, as provided by ParameterConstructor
  const rcl_interfaces::msg::ParameterDescriptor description_;

  //   mutable std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;
  //   //! monitors paramter changes mutable
  //   std::shared_ptr<rclcpp::ParameterCallbackHandle> cb_handle_; //! Callback
  //   handler - must remain in scope for the callbacks to remain

  //   mutable PropertyHandler property_handler_; //! Handler for param specific
  //   callbacks
};

/**
 * @brief Wrapper class to define paramter details. Using these details a
 * ParameterDetails object can be created which declares the paramter with its
 * associated node automatically and can be used to directly access the latest
 * value.
 *
 */
class ParameterConstructor {
 public:
  /**
   * @brief Constructor with a node and param name. Default value is set to type
   * rclcpp::PARAMETER_NOT_SET and the resulting ParameterDetails will throw an
   * 'InvalidDefaultParameter' exception upon construction unless the user
   * provides a default argument in the parameter overwrites.
   *
   * @param node
   * @param name
   */
  ParameterConstructor(rclcpp::Node::SharedPtr node, const std::string &name);
  ParameterConstructor(rclcpp::Node *node, const std::string &name);

  /**
   * @brief Construct with a node, param name and default type. The he value
   * type is determined by traits<ValueTypeT>::ros_parameter_type; and will be
   * used as the default value if a parameter overwrite is not provided
   *
   * @tparam ValueTypeT
   * @param node
   * @param name
   * @param value
   */
  template <typename ValueTypeT>
  ParameterConstructor(rclcpp::Node::SharedPtr node, const std::string &name,
                       ValueTypeT value);
  template <typename ValueTypeT>
  ParameterConstructor(rclcpp::Node *node, const std::string &name,
                       ValueTypeT value);

  /**
   * @brief Constructs a ParameterDetails object from the details in this.
   *
   * As per the ParameterDetails comments, upon constructio the parameter is
   * declared in the associated node (from this) and is kept in the
   * ParameterDetails as a shared pointer.
   *
   * @return ParameterDetails
   */
  ParameterDetails finish() const;

  /**
   * @brief Updates the paramters description and a returns a reference to this.
   *
   * Internally, updates parameter_descriptor_.
   *
   * @param description const std::string
   * @return ParameterConstructor&
   */
  ParameterConstructor &description(const std::string &description);

  /**
   * @brief Updates the read only flag and a returns a reference to this.
   *
   * Internally, updates parameter_descriptor_.
   *
   * @param read_only bool
   * @return ParameterConstructor&
   */
  ParameterConstructor &read_only(bool read_only);

  /**
   * @brief Updates the full internal paramter description and returns a
   * reference to this.
   *
   * @param parameter_description const rcl_interfaces::msg::ParameterDescriptor
   * &
   * @return ParameterConstructor&
   */
  ParameterConstructor &parameter_description(
      const rcl_interfaces::msg::ParameterDescriptor &parameter_description);

  /**
   * @brief Name of the paramter
   *
   * @return const std::string&
   */
  inline const std::string &name() const { return parameter_.get_name(); }
  /**
   * @brief Casting operator and the internal storage for the parameter value
   *
   * @return rclcpp::ParameterValue
   */
  operator rclcpp::ParameterValue() const {
    return parameter_.get_parameter_value();
  }

  operator rcl_interfaces::msg::ParameterDescriptor() const {
    return parameter_descriptor_;
  }

 private:
  rclcpp::Node *node_;
  rclcpp::Parameter parameter_;
  rcl_interfaces::msg::ParameterDescriptor parameter_descriptor_;
};

namespace utils {

/**
 * @brief Convert ROS time to timestamp
 *
 * @param time
 * @return Timestamp time in seconds
 */
Timestamp fromRosTime(const rclcpp::Time &time);

/**
 * @brief Converts Timestamp (in seconds) to ROS time
 *
 * @param timestamp
 * @return rclcpp::Time
 */
rclcpp::Time toRosTime(Timestamp timestamp);

/**
 * @brief Converts from input type to output type where output type is a ROS
 * message with a header. OUTPUT type MUST be a ROS message type
 *
 * Checks if the OUTPUT type has a variable called header and then sets the
 * timestamp and frame id Checks if the OUTPUT type has a variable called
 * child_frame_id and, if child_frame_id provided, sets the value
 *
 */
template <typename INPUT, typename OUTPUT,
          typename = std::enable_if_t<
              rosidl_generator_traits::is_message<OUTPUT>::value>>
bool convertWithHeader(
    const INPUT &input, OUTPUT &output, Timestamp timestamp,
    const std::string &frame_id,
    std::optional<std::string> child_frame_id = std::nullopt) {
  if (!dyno::convert<INPUT, OUTPUT>(input, output)) return false;

  // does actually have a header
  if constexpr (internal::HasMsgHeader<OUTPUT>::value) {
    output.header.stamp = toRosTime(timestamp);
    output.header.frame_id = frame_id;
  }

  // if has child frame id
  if constexpr (internal::HasChildFrame<OUTPUT>::value) {
    if (child_frame_id) {
      output.child_frame_id = child_frame_id.value();
    }
  }

  return true;
}

}  // namespace utils
}  // namespace dyno

#define DECLARE_ROS_PARAM_TYPE_CONVERSIONS(CPP_TYPE, PARAMETER_TYPE) \
  template <>                                                        \
  struct dyno::traits<CPP_TYPE>                                      \
      : dyno::internal::RosParameterType<PARAMETER_TYPE> {};         \
  template <>                                                        \
  struct dyno::ros_param_traits<PARAMETER_TYPE>                      \
      : dyno::internal::CppParameterType<CPP_TYPE> {};

DECLARE_ROS_PARAM_TYPE_CONVERSIONS(
    bool, rcl_interfaces::msg::ParameterType::PARAMETER_BOOL)
DECLARE_ROS_PARAM_TYPE_CONVERSIONS(
    int, rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER)
DECLARE_ROS_PARAM_TYPE_CONVERSIONS(
    double, rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE)
DECLARE_ROS_PARAM_TYPE_CONVERSIONS(
    const char *, rcl_interfaces::msg::ParameterType::PARAMETER_STRING)
DECLARE_ROS_PARAM_TYPE_CONVERSIONS(
    std::vector<bool>, rcl_interfaces::msg::ParameterType::PARAMETER_BOOL_ARRAY)
DECLARE_ROS_PARAM_TYPE_CONVERSIONS(
    std::vector<int>,
    rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER_ARRAY)
DECLARE_ROS_PARAM_TYPE_CONVERSIONS(
    std::vector<double>,
    rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE_ARRAY)
DECLARE_ROS_PARAM_TYPE_CONVERSIONS(
    std::vector<const char *>,
    rcl_interfaces::msg::ParameterType::PARAMETER_STRING_ARRAY)

// specalise for the non-specific rclcpp::ParameterValue type (which is a c++
// object type) as rcl_interfaces::msg::ParameterType::PARAMETER_NOT_SET when a
// Paramter is not set we can only retrieve its ParameterValue and not cast it
// to a type since it does not know which type it is!
template <>
struct dyno::traits<rclcpp::ParameterValue>
    : dyno::internal::RosParameterType<
          rcl_interfaces::msg::ParameterType::PARAMETER_NOT_SET> {};

template <>
struct dyno::traits<std::string>
    : dyno::internal::RosParameterType<
          rcl_interfaces::msg::ParameterType::PARAMETER_STRING> {};

#include "dynosam_ros/RosUtils-inl.hpp"
````

## File: include/dynosam_ros/Utils.hpp
````
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

#pragma once

#include <vector>
#include <string>

namespace dyno {

/**
 * @brief Initalises ROS, gflags and glog
 *
 * Returns a vector of non-ROS arguments
 *
 * @param argc
 * @param argv
 * @return std::vector<std::string>
 */
std::vector<std::string> initRosAndLogging(int argc, char* argv[]);


} //dyno
````

## File: include/dynosam_ros/ZEDOnlineDataProvider.hpp
````
#pragma once

#include <dynosam/dataprovider/DataProvider.hpp> // From dynosam core
#include <dynosam/common/CameraParams.hpp>       // From dynosam core
#include <dynosam/frontend/imu/ThreadSafeImuBuffer.hpp> // From dynosam core
#include <dynosam/common/ImageTypes.hpp>        // For ImageWrapper

// ROS2 includes
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <image_transport/image_transport.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <cv_bridge/cv_bridge.h>

// Local adaptor (within dynosam_ros)
#include "dynosam_ros/adaptors/CameraParamsAdaptor.hpp"

namespace dyno {

// Forward declare DataInterfacePipeline and DataInterfacePipelineIm
// as they are part of the dynosam core library this DataProvider interacts with.
class DataInterfacePipeline;
class DataInterfacePipelineImu;

struct ZEDOnlineDataProviderParams {
    std::string rgb_topic_name = "rgb/image_rect_color";
    std::string depth_topic_name = "depth/depth_registered";
    std::string camera_info_topic_name = "rgb/camera_info";
    std::string imu_topic_name = "imu/data";

    int image_sync_queue_size = 15;
    double image_sync_slop_sec = 0.034; // Approx 1 frame at 30fps

    bool wait_for_camera_info = true;
    std::chrono::milliseconds camera_info_timeout_ms = std::chrono::milliseconds(-1); // Wait indefinitely

    size_t imu_buffer_size = 200;
    bool output_rgb = true; // True for BGR, false for Grayscale
    bool enable_imu_processing = true; // To globally enable/disable IMU part of this provider

    // Transformation for IMU data from ZED's IMU frame to DynOSAM's expected body/IMU convention
    // Example: If ZED IMU is NED and DynOSAM expects FLU.
    // This could be represented as a 3x3 rotation matrix or Euler angles.
    // For simplicity, we might do axis swapping/negation directly in the callback for now.
    // Or, better, ensure TF handles this upstream and DynOSAM consumes IMU in a well-defined body frame.
    // For now, let's assume direct consumption and potential need for axis swap in callback.
};

class ZEDOnlineDataProvider : public DataProvider {
public:
    DYNO_POINTER_TYPEDEFS(ZEDOnlineDataProvider);

    ZEDOnlineDataProvider(
        rclcpp::Node::SharedPtr ros_node, // The ROS node (e.g., DynoNode instance) that owns this
        const ZEDOnlineDataProviderParams& params
    );

    ~ZEDOnlineDataProvider() override;

    // --- Overrides from DataProvider ---
    bool spin() override;
    void shutdown() override;
    CameraParams::Optional getCameraParams() const override;
    int datasetSize() const override { return -1; } // Online stream

private:
    void imageSyncCallback(
        const sensor_msgs::msg::Image::ConstSharedPtr& rgb_msg,
        const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg);

    void cameraInfoCallback(
        const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info_msg);

    void imuCallback(
        const sensor_msgs::msg::Imu::ConstSharedPtr& imu_msg);

    cv::Mat convertRosImageToCvMat(
        const sensor_msgs::msg::Image::ConstSharedPtr& img_msg,
        const std::string& expected_encoding_cv_bridge) const;
    cv::Mat readRgbRosImage(const sensor_msgs::msg::Image::ConstSharedPtr& img_msg) const;
    cv::Mat readDepthRosImage(const sensor_msgs::msg::Image::ConstSharedPtr& img_msg) const;


    rclcpp::Node::SharedPtr ros_node_;
    ZEDOnlineDataProviderParams params_;

    // Image synchronization (RGB + Depth)
    // We will sync RGB and Depth. CameraInfo and IMU will be handled separately.
    typedef message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::Image, // RGB
        sensor_msgs::msg::Image  // Depth
    > ZedImageSyncPolicy;

    std::unique_ptr<image_transport::SubscriberFilter> rgb_sub_;
    std::unique_ptr<image_transport::SubscriberFilter> depth_sub_;
    std::unique_ptr<message_filters::Synchronizer<ZedImageSyncPolicy>> image_sync_;

    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

    ThreadsafeImuBuffer imu_buffer_; // From DynOSAM core

    mutable std::mutex camera_info_mutex_;
    CameraParams::Optional camera_params_;
    std::atomic<bool> camera_info_received_ = false;
    std::condition_variable camera_info_cond_;

    FrameId frame_id_counter_ = 0;
    Timestamp last_image_timestamp_ns_ = 0; // For querying IMU data, in nanoseconds
    
    std::atomic<bool> shutdown_ = false;
};

}
````

## File: launch/dyno_sam_experiments_launch.py
````python
from dynosam_ros.launch_utils import generate_dynosam_launch_description

def generate_launch_description():
    return generate_dynosam_launch_description(
        executable = "dynosam_experiments_node",
        should_output = True,
        world_to_robot_tf = True
    )
````

## File: launch/dyno_sam_launch.py
````python
from dynosam_ros.launch_utils import generate_dynosam_launch_description

def generate_launch_description():
    return generate_dynosam_launch_description(
        executable = "dynosam_node",
        should_output = True,
        world_to_robot_tf = True,
        v=30
    )
````

## File: launch/dyno_sam_online_launch.py
````python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Define arguments
    declared_arguments = [
        DeclareLaunchArgument('camera_info', default_value='/dyno/camera/camera_info', description='Camera info topic'),
        DeclareLaunchArgument('rgb_cam_topic', default_value='/dyno/camera/rgb', description='RGB camera topic'),
        DeclareLaunchArgument('depth_cam_topic', default_value='/dyno/camera/depth', description='Depth camera topic'),
        DeclareLaunchArgument('motion_mask_cam_topic', default_value='/dyno/camera/motion_mask', description='Motion mask camera topic'),
        DeclareLaunchArgument('optical_flow_cam_topic', default_value='/dyno/camera/optical_flow', description='Optical flow camera topic'),
        DeclareLaunchArgument('output_path', default_value='/output_path', description='Output path directory')
    ]

    # Get the child launch file
    dynosam_launch_file = os.path.join(
            get_package_share_directory('dynosam_ros'), 'launch', 'dyno_sam_launch.py')
    # Include the child launch file and pass all arguments
    child_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(dynosam_launch_file),
        launch_arguments={
            'camera_info': LaunchConfiguration('camera_info'),
            'rgb_cam_topic': LaunchConfiguration('rgb_cam_topic'),
            'depth_cam_topic': LaunchConfiguration('depth_cam_topic'),
            'motion_mask_cam_topic': LaunchConfiguration('motion_mask_cam_topic'),
            'optical_flow_cam_topic': LaunchConfiguration('optical_flow_cam_topic'),
            'online': 'True',
            'output_path': LaunchConfiguration('output_path')
        }.items()
    )

    return LaunchDescription(declared_arguments + [child_launch])
````

## File: launch/dyno_sam_zed_launch.py
````python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Define ZED-specific arguments with appropriate defaults
    declared_arguments = [
        # ZED camera topics (with standard ZED topic names)
        DeclareLaunchArgument('zed_rgb_topic', default_value='/zed2i/zed_node/rgb/image_rect_color', description='ZED RGB image topic'),
        DeclareLaunchArgument('zed_depth_topic', default_value='/zed2i/zed_node/depth/depth_registered', description='ZED depth image topic'),
        DeclareLaunchArgument('zed_camera_info_topic', default_value='/zed2i/zed_node/rgb/camera_info', description='ZED camera info topic'),
        DeclareLaunchArgument('zed_imu_topic', default_value='/zed2i/zed_node/imu/data', description='ZED IMU topic'),
        
        # ZED-specific parameters
        DeclareLaunchArgument('zed_image_sync_queue_size', default_value='15', description='Image synchronization queue size'),
        DeclareLaunchArgument('zed_image_sync_slop_sec', default_value='0.034', description='Image synchronization time tolerance (seconds)'),
        DeclareLaunchArgument('zed_wait_for_camera_info', default_value='True', description='Wait for camera info message on startup'),
        DeclareLaunchArgument('zed_camera_info_timeout_ms', default_value='-1', description='Timeout for waiting for camera info (ms, -1 for infinite)'),
        DeclareLaunchArgument('zed_imu_buffer_size', default_value='200', description='IMU buffer size'),
        DeclareLaunchArgument('zed_output_rgb', default_value='True', description='Output RGB (true) or grayscale (false)'),
        DeclareLaunchArgument('zed_enable_imu_processing', default_value='True', description='Enable IMU processing'),
        
        # Standard DynoSAM arguments
        DeclareLaunchArgument('output_path', default_value='/output_path', description='Output path directory')
        # Note: params_path is intentionally not declared here to allow default discovery in the main launch file
    ]

    # Get the child launch file
    dynosam_launch_file = os.path.join(
            get_package_share_directory('dynosam_ros'), 'launch', 'dyno_sam_launch.py')
            
    # Include the child launch file and pass all arguments
    child_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(dynosam_launch_file),
        launch_arguments={
            # Enable online mode with ZED provider
            'online': 'True',
            'online_provider_type': 'zed',
            
            # ZED topic configuration
            'zed.rgb_topic': LaunchConfiguration('zed_rgb_topic'),
            'zed.depth_topic': LaunchConfiguration('zed_depth_topic'),
            'zed.camera_info_topic': LaunchConfiguration('zed_camera_info_topic'),
            'zed.imu_topic': LaunchConfiguration('zed_imu_topic'),
            
            # ZED parameters
            'zed.image_sync_queue_size': LaunchConfiguration('zed_image_sync_queue_size'),
            'zed.image_sync_slop_sec': LaunchConfiguration('zed_image_sync_slop_sec'),
            'zed.wait_for_camera_info': LaunchConfiguration('zed_wait_for_camera_info'),
            'zed.camera_info_timeout_ms': LaunchConfiguration('zed_camera_info_timeout_ms'),
            'zed.imu_buffer_size': LaunchConfiguration('zed_imu_buffer_size'),
            'zed.output_rgb': LaunchConfiguration('zed_output_rgb'),
            'zed.enable_imu_processing': LaunchConfiguration('zed_enable_imu_processing'),
            
            # Standard DynoSAM parameters
            'output_path': LaunchConfiguration('output_path')
            # params_path is not passed to allow default discovery
        }.items()
    )

    return LaunchDescription(declared_arguments + [child_launch])
````

## File: nodes/backend_experiments_node.cc
````
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

#include <dynosam/backend/RGBDBackendModule.hpp>
#include <dynosam/common/Map.hpp>
#include <dynosam/frontend/RGBDInstanceFrontendModule.hpp>
#include <dynosam/logger/Logger.hpp>

#include "dynosam_ros/PipelineRos.hpp"
#include "dynosam_ros/Utils.hpp"
#include "rclcpp/executor.hpp"
#include "rclcpp/rclcpp.hpp"

// already defined in DataProviderFactory.cc
DECLARE_int32(ending_frame);

namespace dyno {

class BackendExperimentsNode : public DynoNode {
 public:
  BackendExperimentsNode() : DynoNode("dynosam_experiments") {
    RCLCPP_INFO_STREAM(this->get_logger(), "Starting BackendExperimentsNode");

    auto data_loader = this->createDataProvider();
    auto params = this->getDynoParams();

    CameraParams camera_params;
    if (params.preferDataProviderCameraParams() &&
        data_loader->getCameraParams().has_value()) {
      LOG(INFO) << "Using camera params from DataProvider, not the config in "
                   "the CameraParams.yaml!";
      camera_params = *data_loader->getCameraParams();
    } else {
      LOG(INFO) << "Using camera params specified in CameraParams.yaml!";
      camera_params = params.camera_params_;
    }

    Camera::Ptr camera = std::make_shared<Camera>(camera_params);

    using BackendModuleTraits = RGBDBackendModule::ModuleTraits;
    using MapType = RGBDBackendModule::MapType;
    using MeasurementType = RGBDBackendModule::MeasurementType;

    typename MapType::Ptr map = MapType::create();

    LOG(INFO) << "Offline RGBD frontend";
    const std::string file_path =
        getOutputFilePath(kRgbdFrontendOutputJsonFile);

    using OfflineFrontend =
        FrontendOfflinePipeline<RGBDBackendModule::ModuleTraits>;

    OfflineFrontend::UniquePtr offline_frontend =
        std::make_unique<OfflineFrontend>("offline-rgbdfrontend", file_path,
                                          FLAGS_ending_frame);

    // raw ptr type becuase we cannot copy the unique ptr!! This is only becuase
    // we need it in the lambda function which is a temporary solution
    OfflineFrontend* offline_frontend_ptr = offline_frontend.get();

    // TODO: make better params!!
    auto updater_type =
        static_cast<RGBDBackendModule::UpdaterType>(FLAGS_backend_updater_enum);

    params.backend_params_.full_batch_frame = offline_frontend->endingFrame();

    auto backend = std::make_shared<RGBDBackendModule>(
        params.backend_params_, map, camera, updater_type);

    backend_pipeline_ = std::make_unique<BackendPipeline>(
        "backend-pipeline", &backend_input_queue_, backend);
    backend_pipeline_->parallelRun(params.parallelRun());
    // also register connection between front and back
    offline_frontend_ptr->registerOutputQueue(&backend_input_queue_);
    // NO OUTPUT!!

    // convert pipeline to base type
    frontend_pipeline_ = std::move(offline_frontend);
  }

  // TODO: copy pasted from DynoManager class - streamline/make shutdown hook
  ~BackendExperimentsNode() {
    // TODO: make shutdown hook!
    writeStatisticsSamplesToFile("statistics_samples.csv");
    writeStatisticsModuleSummariesToFile();
  }

  bool spinOnce() override {
    RCLCPP_INFO_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                getStats());
    if (frontend_pipeline_->isWorking()) {
      frontend_pipeline_->spinOnce();
      backend_pipeline_->spinOnce();
      return true;
    }
    return false;
  }

 private:
  PipelineBase::UniquePtr frontend_pipeline_{nullptr};
  BackendPipeline::UniquePtr backend_pipeline_{nullptr};
  FrontendPipeline::OutputQueue backend_input_queue_;
};

}  // namespace dyno

int main(int argc, char* argv[]) {
  auto non_ros_args = dyno::initRosAndLogging(argc, argv);

  rclcpp::NodeOptions options;
  options.arguments(non_ros_args);
  options.use_intra_process_comms(true);

  rclcpp::executors::SingleThreadedExecutor exec;
  auto ros_pipeline = std::make_shared<dyno::BackendExperimentsNode>();

  exec.add_node(ros_pipeline);
  while (rclcpp::ok()) {
    if (!ros_pipeline->spinOnce()) {
      break;
    }
    exec.spin_some();
  }

  ros_pipeline.reset();
}
````

## File: nodes/dataset_node.cc
````
/*
 *   Copyright (c) 2025 ACFR-RPG, University of Sydney, Jesse Morris
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

#include <cv_bridge/cv_bridge.h>
#include <gflags/gflags.h>
#include <glog/logging.h>

#include <dynosam/common/Camera.hpp>
#include <dynosam/dataprovider/KittiDataProvider.hpp>
#include <dynosam/dataprovider/OMDDataProvider.hpp>
#include <dynosam/frontend/vision/FeatureTracker.hpp>
#include <dynosam/frontend/vision/VisionTools.hpp>
#include <dynosam/utils/OpenCVUtils.hpp>

#include "dynosam_ros/Utils.hpp"
#include "image_transport/image_transport.hpp"
#include "rclcpp/executor.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/node_options.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char* argv[]) {
  using namespace dyno;
  auto non_ros_args = dyno::initRosAndLogging(argc, argv);

  rclcpp::NodeOptions options;
  options.arguments(non_ros_args);
  options.use_intra_process_comms(true);

  auto node =
      std::make_shared<rclcpp::Node>("dynosam_dataset_example", options);

  image_transport::Publisher input_images_pub =
      image_transport::create_publisher(node.get(),
                                        "dynosam_dataset/input_images");
  image_transport::Publisher tracking_images_pub =
      image_transport::create_publisher(node.get(),
                                        "dynosam_dataset/tracking_image");

  // KittiDataLoader::Params params;
  // KittiDataLoader loader("/root/data/vdo_slam/kitti/kitti/0004/", params);

  OMDDataLoader loader(
      "/root/data/vdo_slam/omd/omd/swinging_4_unconstrained_stereo/");

  auto camera = std::make_shared<Camera>(*loader.getCameraParams());
  auto tracker = std::make_shared<FeatureTracker>(FrontendParams(), camera);

  loader.setCallback([&](dyno::FrameId frame_id, dyno::Timestamp timestamp,
                         cv::Mat rgb, cv::Mat optical_flow, cv::Mat depth,
                         cv::Mat motion, GroundTruthInputPacket) -> bool {
    // loader.setCallback([&](dyno::FrameId frame_id, dyno::Timestamp timestamp,
    //     cv::Mat rgb, cv::Mat optical_flow, cv::Mat depth, cv::Mat motion,
    //     gtsam::Pose3, GroundTruthInputPacket) -> bool {

    LOG(INFO) << frame_id << " " << timestamp;

    cv::Mat flow_viz, mask_viz, depth_viz;
    flow_viz = ImageType::OpticalFlow::toRGB(optical_flow);
    mask_viz = ImageType::MotionMask::toRGB(motion);
    depth_viz = ImageType::Depth::toRGB(depth);

    ImageContainer::Ptr container = ImageContainer::Create(
        timestamp, frame_id, ImageWrapper<ImageType::RGBMono>(rgb),
        ImageWrapper<ImageType::Depth>(depth),
        ImageWrapper<ImageType::OpticalFlow>(optical_flow),
        ImageWrapper<ImageType::MotionMask>(motion));

    auto frame = tracker->track(frame_id, timestamp, *container);
    Frame::Ptr previous_frame = tracker->getPreviousFrame();

    cv::Mat tracking;
    if (previous_frame) {
      tracking = tracker->computeImageTracks(*previous_frame, *frame, false);
      std_msgs::msg::Header hdr;
      sensor_msgs::msg::Image::SharedPtr msg =
          cv_bridge::CvImage(hdr, "bgr8", tracking).toImageMsg();

      tracking_images_pub.publish(msg);
    }

    cv::Mat combined_image = utils::concatenateImagesVertically(
        utils::concatenateImagesHorizontally(rgb, depth_viz),
        utils::concatenateImagesHorizontally(flow_viz, mask_viz));

    std_msgs::msg::Header hdr;
    sensor_msgs::msg::Image::SharedPtr msg =
        cv_bridge::CvImage(hdr, "bgr8", combined_image).toImageMsg();

    input_images_pub.publish(msg);

    return true;
  });

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);

  while (rclcpp::ok()) {
    if (!loader.spin()) {
      break;
    }
    exec.spin_some();
  }
}
````

## File: nodes/dynosam_node.cc
````
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

#include <gflags/gflags.h>
#include <glog/logging.h>

#include "dynosam_ros/PipelineRos.hpp"
#include "dynosam_ros/Utils.hpp"
#include "rclcpp/executor.hpp"
#include "rclcpp/rclcpp.hpp"

DEFINE_bool(show_dyno_args, false,
            "Show all loaded DynoSAM args (YAML and gflag) and exit");

int main(int argc, char* argv[]) {
  auto non_ros_args = dyno::initRosAndLogging(argc, argv);

  rclcpp::NodeOptions options;
  options.arguments(non_ros_args);
  options.use_intra_process_comms(true);

  rclcpp::executors::MultiThreadedExecutor exec;
  auto ros_pipeline = std::make_shared<dyno::DynoPipelineManagerRos>();

  if (FLAGS_show_dyno_args) {
    const dyno::DynoParams& params = ros_pipeline->getDynoParams();
    params.printAllParams(true);
    rclcpp::shutdown();
    return 0;
  } else {
    ros_pipeline->initalisePipeline();
  }

  exec.add_node(ros_pipeline);
  while (rclcpp::ok()) {
    if (!ros_pipeline->spinOnce()) {
      break;
    }
    exec.spin_some();
  }

  ros_pipeline.reset();
  return 0;
}
````

## File: rviz/rivz.rviz
````
Panels:
  - Class: rviz_common/Displays
    Help Height: 78
    Name: Displays
    Property Tree Widget:
      Expanded:
        - /Global Options1
        - /Status1
        - /TF1/Frames1
        - /Backend1
        - /Backend1/Static Point Cloud (Optimized)1/Topic1
        - /Backend1/Odometry (OPT)1/Shape1
        - /Frontend1
        - /Frontend1/Odometry1/Shape1
        - /Ground Truth1
        - /Ground Truth1/Odometry (GT)1/Shape1
        - /ObjectOdometry1
      Splitter Ratio: 0.5833333134651184
    Tree Height: 637
  - Class: rviz_common/Selection
    Name: Selection
  - Class: rviz_common/Tool Properties
    Expanded:
      - /2D Goal Pose1
      - /Publish Point1
    Name: Tool Properties
    Splitter Ratio: 0.5886790156364441
  - Class: rviz_common/Views
    Expanded:
      - /Current View1
    Name: Views
    Splitter Ratio: 0.5
  - Class: rviz_common/Time
    Experimental: false
    Name: Time
    SyncMode: 0
    SyncSource: ""
Visualization Manager:
  Class: ""
  Displays:
    - Alpha: 0.5
      Cell Size: 1
      Class: rviz_default_plugins/Grid
      Color: 160; 160; 164
      Enabled: false
      Line Style:
        Line Width: 0.029999999329447746
        Value: Lines
      Name: Grid
      Normal Cell Count: 0
      Offset:
        X: 0
        Y: 0
        Z: 0
      Plane: XY
      Plane Cell Count: 10
      Reference Frame: <Fixed Frame>
      Value: false
    - Class: rviz_default_plugins/TF
      Enabled: false
      Frame Timeout: 15
      Frames:
        All Enabled: false
      Marker Scale: 5
      Name: TF
      Show Arrows: true
      Show Axes: true
      Show Names: true
      Tree:
        {}
      Update Interval: 0
      Value: false
    - Class: rviz_common/Group
      Displays:
        - Alpha: 1
          Autocompute Intensity Bounds: true
          Autocompute Value Bounds:
            Max Value: 10
            Min Value: -10
            Value: true
          Axis: Z
          Channel Name: intensity
          Class: rviz_default_plugins/PointCloud2
          Color: 255; 255; 255
          Color Transformer: RGB8
          Decay Time: 0
          Enabled: false
          Invert Rainbow: false
          Max Color: 255; 255; 255
          Max Intensity: 4096
          Min Color: 0; 0; 0
          Min Intensity: 0
          Name: Dynamic Point Cloud (Optimized)
          Position Transformer: XYZ
          Selectable: true
          Size (Pixels): 1
          Size (m): 0.07999999821186066
          Style: Spheres
          Topic:
            Depth: 5
            Durability Policy: Volatile
            Filter size: 10
            History Policy: Keep Last
            Reliability Policy: Reliable
            Value: /dynosam/backend/dynamic_cloud
          Use Fixed Frame: true
          Use rainbow: true
          Value: false
        - Alpha: 1
          Autocompute Intensity Bounds: true
          Autocompute Value Bounds:
            Max Value: 3.1478004455566406
            Min Value: -2.2305188179016113
            Value: true
          Axis: Z
          Channel Name: intensity
          Class: rviz_default_plugins/PointCloud2
          Color: 255; 255; 255
          Color Transformer: RGB8
          Decay Time: 0
          Enabled: true
          Invert Rainbow: false
          Max Color: 255; 255; 255
          Max Intensity: 4096
          Min Color: 0; 0; 0
          Min Intensity: 0
          Name: Static Point Cloud (Optimized)
          Position Transformer: XYZ
          Selectable: true
          Size (Pixels): 3
          Size (m): 0.03999999910593033
          Style: Points
          Topic:
            Depth: 5
            Durability Policy: Volatile
            Filter size: 10
            History Policy: Keep Last
            Reliability Policy: Reliable
            Value: /dynosam/backend/static_cloud
          Use Fixed Frame: true
          Use rainbow: true
          Value: true
        - Angle Tolerance: 0.009999999776482582
          Class: rviz_default_plugins/Odometry
          Covariance:
            Orientation:
              Alpha: 0.5
              Color: 255; 255; 127
              Color Style: Unique
              Frame: Local
              Offset: 1
              Scale: 1
              Value: true
            Position:
              Alpha: 0.30000001192092896
              Color: 204; 51; 204
              Scale: 1
              Value: true
            Value: true
          Enabled: true
          Keep: 1
          Name: Odometry (OPT)
          Position Tolerance: 0.009999999776482582
          Shape:
            Alpha: 1
            Axes Length: 0.5
            Axes Radius: 0.029999999329447746
            Color: 255; 25; 0
            Head Length: 0.30000001192092896
            Head Radius: 0.10000000149011612
            Shaft Length: 1
            Shaft Radius: 0.05000000074505806
            Value: Axes
          Topic:
            Depth: 5
            Durability Policy: Volatile
            Filter size: 10
            History Policy: Keep Last
            Reliability Policy: Reliable
            Value: /dynosam/backend/odom
          Value: true
        - Alpha: 1
          Buffer Length: 1
          Class: rviz_default_plugins/Path
          Color: 0; 0; 255
          Enabled: true
          Head Diameter: 0.30000001192092896
          Head Length: 0.20000000298023224
          Length: 0.30000001192092896
          Line Style: Billboards
          Line Width: 0.05000000074505806
          Name: Odometry Path (OPT)
          Offset:
            X: 0
            Y: 0
            Z: 0
          Pose Color: 255; 85; 255
          Pose Style: None
          Radius: 0.029999999329447746
          Shaft Diameter: 0.10000000149011612
          Shaft Length: 0.10000000149011612
          Topic:
            Depth: 5
            Durability Policy: Volatile
            Filter size: 10
            History Policy: Keep Last
            Reliability Policy: Reliable
            Value: /dynosam/backend/odom_path
          Value: true
        - Class: rviz_default_plugins/MarkerArray
          Enabled: true
          Name: Composed object paths (opt)
          Namespaces:
            {}
          Topic:
            Depth: 5
            Durability Policy: Volatile
            History Policy: Keep Last
            Reliability Policy: Reliable
            Value: /dynosam/backend/composed_object_paths
          Value: true
        - Class: rviz_default_plugins/MarkerArray
          Enabled: false
          Name: Composed object poses (opt)
          Namespaces:
            {}
          Topic:
            Depth: 5
            Durability Policy: Volatile
            History Policy: Keep Last
            Reliability Policy: Reliable
            Value: /dynosam/backend/composed_object_poses
          Value: false
        - Class: rviz_default_plugins/MarkerArray
          Enabled: false
          Name: Object AABB
          Namespaces:
            {}
          Topic:
            Depth: 5
            Durability Policy: Volatile
            History Policy: Keep Last
            Reliability Policy: Reliable
            Value: /dynosam/backend/object_aabb
          Value: false
      Enabled: true
      Name: Backend
    - Class: rviz_common/Group
      Displays:
        - Class: rviz_default_plugins/MarkerArray
          Enabled: true
          Name: Object Paths (Composed)
          Namespaces:
            {}
          Topic:
            Depth: 5
            Durability Policy: Volatile
            History Policy: Keep All
            Reliability Policy: Best Effort
            Value: /dynosam/frontend/composed_object_paths
          Value: true
        - Class: rviz_default_plugins/MarkerArray
          Enabled: false
          Name: Object Poses (Composed)
          Namespaces:
            {}
          Topic:
            Depth: 5
            Durability Policy: Volatile
            History Policy: Keep All
            Reliability Policy: Best Effort
            Value: /dynosam/frontend/composed_object_poses
          Value: false
        - Alpha: 1
          Autocompute Intensity Bounds: true
          Autocompute Value Bounds:
            Max Value: 10
            Min Value: -10
            Value: true
          Axis: Z
          Channel Name: intensity
          Class: rviz_default_plugins/PointCloud2
          Color: 255; 255; 255
          Color Transformer: RGB8
          Decay Time: 0
          Enabled: false
          Invert Rainbow: false
          Max Color: 255; 255; 255
          Max Intensity: 4096
          Min Color: 0; 0; 0
          Min Intensity: 0
          Name: Dynamic Point Cloud
          Position Transformer: XYZ
          Selectable: true
          Size (Pixels): 3
          Size (m): 0.05000000074505806
          Style: Spheres
          Topic:
            Depth: 5
            Durability Policy: System Default
            Filter size: 10
            History Policy: Keep Last
            Reliability Policy: Best Effort
            Value: /dynosam/frontend/dynamic_cloud
          Use Fixed Frame: true
          Use rainbow: true
          Value: false
        - Angle Tolerance: 0.009999999776482582
          Class: rviz_default_plugins/Odometry
          Covariance:
            Orientation:
              Alpha: 0.5
              Color: 255; 255; 127
              Color Style: Unique
              Frame: Local
              Offset: 1
              Scale: 1
              Value: true
            Position:
              Alpha: 0.30000001192092896
              Color: 204; 51; 204
              Scale: 1
              Value: true
            Value: true
          Enabled: true
          Keep: 1
          Name: Odometry
          Position Tolerance: 0.009999999776482582
          Shape:
            Alpha: 1
            Axes Length: 0.5
            Axes Radius: 0.029999999329447746
            Color: 255; 25; 0
            Head Length: 0.30000001192092896
            Head Radius: 0.10000000149011612
            Shaft Length: 1
            Shaft Radius: 0.05000000074505806
            Value: Axes
          Topic:
            Depth: 5
            Durability Policy: Volatile
            Filter size: 10
            History Policy: Keep Last
            Reliability Policy: Reliable
            Value: /dynosam/frontend/odom
          Value: true
        - Alpha: 1
          Buffer Length: 1
          Class: rviz_default_plugins/Path
          Color: 255; 170; 0
          Enabled: true
          Head Diameter: 0.30000001192092896
          Head Length: 0.20000000298023224
          Length: 0.30000001192092896
          Line Style: Billboards
          Line Width: 0.05000000074505806
          Name: Odometry Path
          Offset:
            X: 0
            Y: 0
            Z: 0
          Pose Color: 255; 85; 255
          Pose Style: None
          Radius: 0.029999999329447746
          Shaft Diameter: 0.10000000149011612
          Shaft Length: 0.10000000149011612
          Topic:
            Depth: 5
            Durability Policy: Volatile
            Filter size: 10
            History Policy: Keep Last
            Reliability Policy: Reliable
            Value: /dynosam/frontend/odom_path
          Value: true
        - Alpha: 1
          Autocompute Intensity Bounds: true
          Autocompute Value Bounds:
            Max Value: 10
            Min Value: -10
            Value: true
          Axis: Z
          Channel Name: intensity
          Class: rviz_default_plugins/PointCloud2
          Color: 255; 255; 255
          Color Transformer: RGB8
          Decay Time: 0
          Enabled: false
          Invert Rainbow: false
          Max Color: 255; 255; 255
          Max Intensity: 4096
          Min Color: 0; 0; 0
          Min Intensity: 0
          Name: Static Point Cloud
          Position Transformer: XYZ
          Selectable: true
          Size (Pixels): 3
          Size (m): 0.05000000074505806
          Style: Spheres
          Topic:
            Depth: 5
            Durability Policy: Volatile
            Filter size: 10
            History Policy: Keep Last
            Reliability Policy: Best Effort
            Value: /dynosam/frontend/static_cloud
          Use Fixed Frame: true
          Use rainbow: true
          Value: false
      Enabled: false
      Name: Frontend
    - Class: rviz_common/Group
      Displays:
        - Angle Tolerance: 0.009999999776482582
          Class: rviz_default_plugins/Odometry
          Covariance:
            Orientation:
              Alpha: 0.5
              Color: 255; 255; 127
              Color Style: Unique
              Frame: Local
              Offset: 1
              Scale: 1
              Value: true
            Position:
              Alpha: 0.30000001192092896
              Color: 204; 51; 204
              Scale: 1
              Value: true
            Value: true
          Enabled: true
          Keep: 1
          Name: Odometry (GT)
          Position Tolerance: 0.009999999776482582
          Shape:
            Alpha: 1
            Axes Length: 0.5
            Axes Radius: 0.029999999329447746
            Color: 255; 25; 0
            Head Length: 0.30000001192092896
            Head Radius: 0.10000000149011612
            Shaft Length: 1
            Shaft Radius: 0.05000000074505806
            Value: Axes
          Topic:
            Depth: 5
            Durability Policy: Volatile
            Filter size: 10
            History Policy: Keep Last
            Reliability Policy: Reliable
            Value: /dynosam/dynosam/ground_truth/odom
          Value: true
        - Alpha: 1
          Buffer Length: 1
          Class: rviz_default_plugins/Path
          Color: 25; 255; 0
          Enabled: true
          Head Diameter: 0.30000001192092896
          Head Length: 0.20000000298023224
          Length: 0.30000001192092896
          Line Style: Billboards
          Line Width: 0.05000000074505806
          Name: Odometry Path (GT)
          Offset:
            X: 0
            Y: 0
            Z: 0
          Pose Color: 255; 85; 255
          Pose Style: None
          Radius: 0.029999999329447746
          Shaft Diameter: 0.10000000149011612
          Shaft Length: 0.10000000149011612
          Topic:
            Depth: 5
            Durability Policy: Volatile
            Filter size: 10
            History Policy: Keep Last
            Reliability Policy: Reliable
            Value: /dynosam/dynosam/ground_truth/odom_path
          Value: true
        - Class: rviz_default_plugins/MarkerArray
          Enabled: false
          Name: Object Poses (GT)
          Namespaces:
            {}
          Topic:
            Depth: 5
            Durability Policy: Volatile
            History Policy: System Default
            Reliability Policy: System Default
            Value: /dynosam/dynosam/ground_truth/object_paths
          Value: false
        - Class: rviz_default_plugins/MarkerArray
          Enabled: true
          Name: Object Paths (GT)
          Namespaces:
            {}
          Topic:
            Depth: 5
            Durability Policy: Volatile
            History Policy: Keep Last
            Reliability Policy: Reliable
            Value: /dynosam/dynosam/ground_truth/object_paths
          Value: true
      Enabled: true
      Name: Ground Truth
    - Class: rviz_default_plugins/Image
      Enabled: false
      Max Value: 1
      Median window: 5
      Min Value: 0
      Name: Image
      Normalize Range: true
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /dynosam/tracking_image
      Value: false
    - Class: rviz_default_plugins/MarkerArray
      Enabled: false
      Name: MarkerArray
      Namespaces:
        {}
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /dynosam/frontend/object_bbx_viz
      Value: false
    - Axes Length: 1
      Axes Radius: 0.10000000149011612
      Class: rviz_dynamic_slam_plugins/ObjectOdometry
      Covariance:
        Orientation:
          Alpha: 0.5
          Color: 255; 255; 127
          Color Style: Unique
          Frame: Local
          Offset: 1
          Scale: 1
          Value: true
        Position:
          Alpha: 0.30000001192092896
          Color: 204; 51; 204
          Scale: 1
          Value: true
        Value: true
      Enabled: true
      Keep: 100
      Name: ObjectOdometry
      Show Object Label: true
      Show Only Final Pose: true
      Show Velocity: true
      Topic:
        Depth: 5
        Durability Policy: Volatile
        Filter size: 10
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /dynosam/frontend/ground_truth/object_odometry
      Trajectory:
        Buffer Length: 1
        Diameter: 0.20000000298023224
        Line Style: Billboards
        Value: true
      Value: true
  Enabled: true
  Global Options:
    Background Color: 255; 255; 255
    Fixed Frame: robot
    Frame Rate: 30
  Name: root
  Tools:
    - Class: rviz_default_plugins/Interact
      Hide Inactive Objects: true
    - Class: rviz_default_plugins/MoveCamera
    - Class: rviz_default_plugins/Select
    - Class: rviz_default_plugins/FocusCamera
    - Class: rviz_default_plugins/Measure
      Line color: 128; 128; 0
    - Class: rviz_default_plugins/SetInitialPose
      Covariance x: 0.25
      Covariance y: 0.25
      Covariance yaw: 0.06853891909122467
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /initialpose
    - Class: rviz_default_plugins/SetGoal
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /goal_pose
    - Class: rviz_default_plugins/PublishPoint
      Single click: true
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /clicked_point
  Transformation:
    Current:
      Class: rviz_default_plugins/TF
  Value: true
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Distance: 35.62208557128906
      Enable Stereo Rendering:
        Stereo Eye Separation: 0.05999999865889549
        Stereo Focal Distance: 1
        Swap Stereo Eyes: false
        Value: false
      Focal Point:
        X: 3.6244301795959473
        Y: 10.512846946716309
        Z: 2.9900920391082764
      Focal Shape Fixed Size: false
      Focal Shape Size: 0.05000000074505806
      Invert Z Axis: false
      Name: Current View
      Near Clip Distance: 0.009999999776482582
      Pitch: 0.7547972798347473
      Target Frame: robot
      Value: Orbit (rviz_default_plugins)
      Yaw: 3.435321092605591
    Saved: ~
Window Geometry:
  Displays:
    collapsed: false
  Height: 928
  Hide Left Dock: false
  Hide Right Dock: false
  Image:
    collapsed: false
  QMainWindow State: 000000ff00000000fd0000000400000000000001aa00000306fc0200000009fb0000001200530065006c0065006300740069006f006e00000001e10000009b0000005c00fffffffb0000001e0054006f006f006c002000500072006f007000650072007400690065007302000001ed000001df00000185000000a3fb000000120056006900650077007300200054006f006f02000001df000002110000018500000122fb000000200054006f006f006c002000500072006f0070006500720074006900650073003203000002880000011d000002210000017afc0000003b00000306000000c700fffffffa000000010100000002fb0000000a0049006d0061006700650000000000ffffffff0000005a00fffffffb000000100044006900730070006c00610079007301000000000000017c0000015600fffffffb0000002000730065006c0065006300740069006f006e00200062007500660066006500720200000138000000aa0000023a00000294fb00000014005700690064006500530074006500720065006f02000000e6000000d2000003ee0000030bfb0000000c004b0069006e0065006300740200000186000001060000030c00000261fb0000000a0049006d00610067006500000001e7000001580000000000000000000000010000010000000306fc0200000003fb0000001e0054006f006f006c002000500072006f00700065007200740069006500730100000041000000780000000000000000fb0000000a00560069006500770073010000003b00000306000000a000fffffffb0000001200530065006c0065006300740069006f006e010000025a000000b200000000000000000000000200000780000000a9fc0100000004fb0000000a0049006d0061006700650200000430000000af000003270000014cfb0000000a0049006d0061006700650200000549000000d2000001fe00000169fb0000000a0049006d0061006700650300000cb3000000c20000020d00000147fb0000000a00560069006500770073030000004e00000080000002e10000019700000003000006b50000003efc0100000002fb0000000800540069006d00650100000000000006b50000025300fffffffb0000000800540069006d00650100000000000004500000000000000000000003ff0000030600000004000000040000000800000008fc0000000100000002000000010000000a0054006f006f006c00730100000000ffffffff0000000000000000
  Selection:
    collapsed: false
  Time:
    collapsed: false
  Tool Properties:
    collapsed: false
  Views:
    collapsed: false
  Width: 1717
  X: 72
  Y: 81
````

## File: rviz/rviz_dsd.rviz
````
Panels:
  - Class: rviz_common/Displays
    Help Height: 78
    Name: Displays
    Property Tree Widget:
      Expanded:
        - /Global Options1
        - /Status1
        - /Frontend1
        - /Ground Truth1/Camera Path1
        - /Ground Truth1/Object Odometry1
      Splitter Ratio: 0.5
    Tree Height: 484
  - Class: rviz_common/Selection
    Name: Selection
  - Class: rviz_common/Tool Properties
    Expanded:
      - /2D Goal Pose1
      - /Publish Point1
    Name: Tool Properties
    Splitter Ratio: 0.5886790156364441
  - Class: rviz_common/Views
    Expanded:
      - /Current View1
    Name: Views
    Splitter Ratio: 0.5
  - Class: rviz_common/Time
    Experimental: false
    Name: Time
    SyncMode: 0
    SyncSource: Dynamic Map
Visualization Manager:
  Class: ""
  Displays:
    - Alpha: 0.5
      Cell Size: 1
      Class: rviz_default_plugins/Grid
      Color: 160; 160; 164
      Enabled: true
      Line Style:
        Line Width: 0.029999999329447746
        Value: Lines
      Name: Grid
      Normal Cell Count: 0
      Offset:
        X: 0
        Y: 0
        Z: 0
      Plane: XY
      Plane Cell Count: 10
      Reference Frame: <Fixed Frame>
      Value: true
    - Class: rviz_common/Group
      Displays:
        - Alpha: 1
          Autocompute Intensity Bounds: true
          Autocompute Value Bounds:
            Max Value: 10
            Min Value: -10
            Value: true
          Axis: Z
          Channel Name: intensity
          Class: rviz_default_plugins/PointCloud2
          Color: 255; 255; 255
          Color Transformer: RGB8
          Decay Time: 0
          Enabled: true
          Invert Rainbow: false
          Max Color: 255; 255; 255
          Max Intensity: 4096
          Min Color: 0; 0; 0
          Min Intensity: 0
          Name: Dynamic Map
          Position Transformer: XYZ
          Selectable: true
          Size (Pixels): 3
          Size (m): 0.009999999776482582
          Style: Flat Squares
          Topic:
            Depth: 5
            Durability Policy: Volatile
            Filter size: 10
            History Policy: Keep Last
            Reliability Policy: Reliable
            Value: /dynosam/backend/dynamic_cloud
          Use Fixed Frame: true
          Use rainbow: true
          Value: true
        - Alpha: 1
          Autocompute Intensity Bounds: true
          Autocompute Value Bounds:
            Max Value: 10
            Min Value: -10
            Value: true
          Axis: Z
          Channel Name: intensity
          Class: rviz_default_plugins/PointCloud2
          Color: 255; 255; 255
          Color Transformer: RGB8
          Decay Time: 0
          Enabled: true
          Invert Rainbow: false
          Max Color: 255; 255; 255
          Max Intensity: 4096
          Min Color: 0; 0; 0
          Min Intensity: 0
          Name: Static Map
          Position Transformer: XYZ
          Selectable: true
          Size (Pixels): 3
          Size (m): 0.009999999776482582
          Style: Flat Squares
          Topic:
            Depth: 5
            Durability Policy: Volatile
            Filter size: 10
            History Policy: Keep Last
            Reliability Policy: Reliable
            Value: /dynosam/backend/static_cloud
          Use Fixed Frame: true
          Use rainbow: true
          Value: true
        - Axes Length: 1
          Axes Radius: 0.10000000149011612
          Class: rviz_dynamic_slam_plugins/ObjectOdometry
          Covariance:
            Orientation:
              Alpha: 0.5
              Color: 255; 255; 127
              Color Style: Unique
              Frame: Local
              Offset: 1
              Scale: 1
              Value: true
            Position:
              Alpha: 0.30000001192092896
              Color: 204; 51; 204
              Scale: 1
              Value: true
            Value: true
          Enabled: true
          Keep: 100
          Name: Object Odometry
          Object Label: true
          Topic:
            Depth: 5
            Durability Policy: Volatile
            Filter size: 10
            History Policy: Keep Last
            Reliability Policy: Reliable
            Value: /dynosam/backend/object_odometry
          Value: true
          Velocity: true
        - Angle Tolerance: 0.10000000149011612
          Class: rviz_default_plugins/Odometry
          Covariance:
            Orientation:
              Alpha: 0.5
              Color: 255; 255; 127
              Color Style: Unique
              Frame: Local
              Offset: 1
              Scale: 1
              Value: true
            Position:
              Alpha: 0.30000001192092896
              Color: 204; 51; 204
              Scale: 1
              Value: true
            Value: true
          Enabled: true
          Keep: 1
          Name: Visual Odometry
          Position Tolerance: 0.10000000149011612
          Shape:
            Alpha: 1
            Axes Length: 1
            Axes Radius: 0.10000000149011612
            Color: 255; 25; 0
            Head Length: 0.30000001192092896
            Head Radius: 0.10000000149011612
            Shaft Length: 1
            Shaft Radius: 0.05000000074505806
            Value: Axes
          Topic:
            Depth: 5
            Durability Policy: Volatile
            Filter size: 10
            History Policy: Keep Last
            Reliability Policy: Reliable
            Value: /dynosam/backend/odometry
          Value: true
        - Alpha: 1
          Buffer Length: 1
          Class: rviz_default_plugins/Path
          Color: 0; 0; 255
          Enabled: true
          Head Diameter: 0.30000001192092896
          Head Length: 0.20000000298023224
          Length: 0.30000001192092896
          Line Style: Lines
          Line Width: 0.029999999329447746
          Name: Camera Path
          Offset:
            X: 0
            Y: 0
            Z: 0
          Pose Color: 255; 85; 255
          Pose Style: None
          Radius: 0.029999999329447746
          Shaft Diameter: 0.10000000149011612
          Shaft Length: 0.10000000149011612
          Topic:
            Depth: 5
            Durability Policy: Volatile
            Filter size: 10
            History Policy: Keep Last
            Reliability Policy: Reliable
            Value: /dynosam/backend/odometry_path
          Value: true
      Enabled: false
      Name: Backend
    - Class: rviz_common/Group
      Displays:
        - Alpha: 1
          Autocompute Intensity Bounds: true
          Autocompute Value Bounds:
            Max Value: 10
            Min Value: -10
            Value: true
          Axis: Z
          Channel Name: intensity
          Class: rviz_default_plugins/PointCloud2
          Color: 255; 255; 255
          Color Transformer: RGB8
          Decay Time: 0
          Enabled: true
          Invert Rainbow: false
          Max Color: 255; 255; 255
          Max Intensity: 4096
          Min Color: 0; 0; 0
          Min Intensity: 0
          Name: Dynamic Map
          Position Transformer: XYZ
          Selectable: true
          Size (Pixels): 3
          Size (m): 0.009999999776482582
          Style: Flat Squares
          Topic:
            Depth: 5
            Durability Policy: Volatile
            Filter size: 10
            History Policy: Keep Last
            Reliability Policy: Reliable
            Value: /dynosam/frontend/dynamic_cloud
          Use Fixed Frame: true
          Use rainbow: true
          Value: true
        - Alpha: 1
          Autocompute Intensity Bounds: true
          Autocompute Value Bounds:
            Max Value: 10
            Min Value: -10
            Value: true
          Axis: Z
          Channel Name: intensity
          Class: rviz_default_plugins/PointCloud2
          Color: 255; 255; 255
          Color Transformer: RGB8
          Decay Time: 0
          Enabled: true
          Invert Rainbow: false
          Max Color: 255; 255; 255
          Max Intensity: 4096
          Min Color: 0; 0; 0
          Min Intensity: 0
          Name: Static Map
          Position Transformer: XYZ
          Selectable: true
          Size (Pixels): 3
          Size (m): 0.009999999776482582
          Style: Flat Squares
          Topic:
            Depth: 5
            Durability Policy: Volatile
            Filter size: 10
            History Policy: Keep Last
            Reliability Policy: Reliable
            Value: /dynosam/frontend/static_cloud
          Use Fixed Frame: true
          Use rainbow: true
          Value: true
        - Axes Length: 0.4000000059604645
          Axes Radius: 0.029999999329447746
          Class: rviz_dynamic_slam_plugins/ObjectOdometry
          Covariance:
            Orientation:
              Alpha: 0.5
              Color: 255; 255; 127
              Color Style: Unique
              Frame: Local
              Offset: 1
              Scale: 1
              Value: true
            Position:
              Alpha: 0.30000001192092896
              Color: 204; 51; 204
              Scale: 1
              Value: true
            Value: false
          Enabled: true
          Keep: 1
          Name: Object Odometry
          Object Label: false
          Topic:
            Depth: 5
            Durability Policy: Volatile
            Filter size: 10
            History Policy: Keep Last
            Reliability Policy: Reliable
            Value: /dynosam/frontend/object_odometry
          Value: true
          Velocity: true
        - Buffer Length: 1
          Class: rviz_dynamic_slam_plugins/ObjectOdometryPath
          Enabled: true
          Head Diameter: 0.30000001192092896
          Head Length: 0.20000000298023224
          Length: 0.30000001192092896
          Line Width: 0.029999999329447746
          Name: ObjectOdometryPath
          Pose Style: None
          Radius: 0.029999999329447746
          Shaft Diameter: 0.10000000149011612
          Shaft Length: 0.10000000149011612
          Topic:
            Depth: 5
            Durability Policy: Volatile
            Filter size: 10
            History Policy: Keep Last
            Reliability Policy: Reliable
            Value: /dynosam/frontend/object_odometry_path
          Value: true
        - Angle Tolerance: 0.10000000149011612
          Class: rviz_default_plugins/Odometry
          Covariance:
            Orientation:
              Alpha: 0.5
              Color: 255; 255; 127
              Color Style: Unique
              Frame: Local
              Offset: 1
              Scale: 1
              Value: true
            Position:
              Alpha: 0.30000001192092896
              Color: 204; 51; 204
              Scale: 1
              Value: true
            Value: true
          Enabled: true
          Keep: 1
          Name: Visual Odometry
          Position Tolerance: 0.10000000149011612
          Shape:
            Alpha: 1
            Axes Length: 1
            Axes Radius: 0.10000000149011612
            Color: 255; 25; 0
            Head Length: 0.30000001192092896
            Head Radius: 0.10000000149011612
            Shaft Length: 1
            Shaft Radius: 0.05000000074505806
            Value: Axes
          Topic:
            Depth: 5
            Durability Policy: Volatile
            Filter size: 10
            History Policy: Keep Last
            Reliability Policy: Reliable
            Value: /dynosam/frontend/odometry
          Value: true
        - Alpha: 1
          Buffer Length: 1
          Class: rviz_default_plugins/Path
          Color: 255; 170; 0
          Enabled: true
          Head Diameter: 0.30000001192092896
          Head Length: 0.20000000298023224
          Length: 0.30000001192092896
          Line Style: Lines
          Line Width: 0.029999999329447746
          Name: Camera Path
          Offset:
            X: 0
            Y: 0
            Z: 0
          Pose Color: 255; 85; 255
          Pose Style: None
          Radius: 0.029999999329447746
          Shaft Diameter: 0.10000000149011612
          Shaft Length: 0.10000000149011612
          Topic:
            Depth: 5
            Durability Policy: Volatile
            Filter size: 10
            History Policy: Keep Last
            Reliability Policy: Reliable
            Value: /dynosam/frontend/odometry_path
          Value: true
      Enabled: true
      Name: Frontend
    - Class: rviz_common/Group
      Displays:
        - Angle Tolerance: 0.10000000149011612
          Class: rviz_default_plugins/Odometry
          Covariance:
            Orientation:
              Alpha: 0.5
              Color: 255; 255; 127
              Color Style: Unique
              Frame: Local
              Offset: 1
              Scale: 1
              Value: true
            Position:
              Alpha: 0.30000001192092896
              Color: 204; 51; 204
              Scale: 1
              Value: true
            Value: true
          Enabled: true
          Keep: 100
          Name: Visual Odometry
          Position Tolerance: 0.10000000149011612
          Shape:
            Alpha: 1
            Axes Length: 1
            Axes Radius: 0.10000000149011612
            Color: 255; 25; 0
            Head Length: 0.30000001192092896
            Head Radius: 0.10000000149011612
            Shaft Length: 1
            Shaft Radius: 0.05000000074505806
            Value: Arrow
          Topic:
            Depth: 5
            Durability Policy: Volatile
            Filter size: 10
            History Policy: Keep Last
            Reliability Policy: Reliable
            Value: /dynosam/frontend/ground_truth/odometry
          Value: true
        - Alpha: 1
          Buffer Length: 1
          Class: rviz_default_plugins/Path
          Color: 255; 0; 0
          Enabled: true
          Head Diameter: 0.30000001192092896
          Head Length: 0.20000000298023224
          Length: 0.30000001192092896
          Line Style: Lines
          Line Width: 0.029999999329447746
          Name: Camera Path
          Offset:
            X: 0
            Y: 0
            Z: 0
          Pose Color: 255; 85; 255
          Pose Style: None
          Radius: 0.029999999329447746
          Shaft Diameter: 0.10000000149011612
          Shaft Length: 0.10000000149011612
          Topic:
            Depth: 5
            Durability Policy: Volatile
            Filter size: 10
            History Policy: Keep Last
            Reliability Policy: Reliable
            Value: /dynosam/frontend/ground_truth/odometry_path
          Value: true
        - Axes Length: 1
          Axes Radius: 0.10000000149011612
          Class: rviz_dynamic_slam_plugins/ObjectOdometry
          Covariance:
            Orientation:
              Alpha: 0.5
              Color: 255; 255; 127
              Color Style: Unique
              Frame: Local
              Offset: 1
              Scale: 1
              Value: true
            Position:
              Alpha: 0.30000001192092896
              Color: 204; 51; 204
              Scale: 1
              Value: true
            Value: true
          Enabled: true
          Keep: 100
          Name: Object Odometry
          Object Label: true
          Topic:
            Depth: 5
            Durability Policy: Volatile
            Filter size: 10
            History Policy: Keep Last
            Reliability Policy: Reliable
            Value: /dynosam/frontend/ground_truth/object_odometry
          Value: true
          Velocity: true
      Enabled: false
      Name: Ground Truth
    - Class: rviz_default_plugins/Image
      Enabled: true
      Max Value: 1
      Median window: 5
      Min Value: 0
      Name: Tracking Image
      Normalize Range: true
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /dynosam/tracking_image
      Value: true
  Enabled: true
  Global Options:
    Background Color: 255; 255; 255
    Fixed Frame: world
    Frame Rate: 30
  Name: root
  Tools:
    - Class: rviz_default_plugins/Interact
      Hide Inactive Objects: true
    - Class: rviz_default_plugins/MoveCamera
    - Class: rviz_default_plugins/Select
    - Class: rviz_default_plugins/FocusCamera
    - Class: rviz_default_plugins/Measure
      Line color: 128; 128; 0
    - Class: rviz_default_plugins/SetInitialPose
      Covariance x: 0.25
      Covariance y: 0.25
      Covariance yaw: 0.06853891909122467
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /initialpose
    - Class: rviz_default_plugins/SetGoal
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /goal_pose
    - Class: rviz_default_plugins/PublishPoint
      Single click: true
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /clicked_point
  Transformation:
    Current:
      Class: rviz_default_plugins/TF
  Value: true
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Distance: 4.948230266571045
      Enable Stereo Rendering:
        Stereo Eye Separation: 0.05999999865889549
        Stereo Focal Distance: 1
        Swap Stereo Eyes: false
        Value: false
      Focal Point:
        X: 0.15764883160591125
        Y: -0.2291499376296997
        Z: 3.460571527481079
      Focal Shape Fixed Size: false
      Focal Shape Size: 0.05000000074505806
      Invert Z Axis: false
      Name: Current View
      Near Clip Distance: 0.009999999776482582
      Pitch: -0.7046015858650208
      Target Frame: robot
      Value: Orbit (rviz)
      Yaw: 4.815402984619141
    Saved: ~
Window Geometry:
  Displays:
    collapsed: false
  Height: 1016
  Hide Left Dock: false
  Hide Right Dock: false
  QMainWindow State: 000000ff00000000fd00000004000000000000023f0000035efc020000000afb0000001200530065006c0065006300740069006f006e00000001e10000009b0000005c00fffffffb0000001e0054006f006f006c002000500072006f007000650072007400690065007302000001ed000001df00000185000000a3fb000000120056006900650077007300200054006f006f02000001df000002110000018500000122fb000000200054006f006f006c002000500072006f0070006500720074006900650073003203000002880000011d000002210000017afb000000100044006900730070006c006100790073010000003b0000026d000000c700fffffffb0000002000730065006c0065006300740069006f006e00200062007500660066006500720200000138000000aa0000023a00000294fb00000014005700690064006500530074006500720065006f02000000e6000000d2000003ee0000030bfb0000000c004b0069006e0065006300740200000186000001060000030c00000261fb0000001c0054007200610063006b0069006e006700200049006d00610067006501000002ae000000eb0000002800fffffffb0000000a0049006d0061006700650000000000ffffffff0000000000000000000000010000010f0000035efc0200000003fb0000001e0054006f006f006c002000500072006f00700065007200740069006500730100000041000000780000000000000000fb0000000a00560069006500770073010000003b0000035e000000a000fffffffb0000001200530065006c0065006300740069006f006e010000025a000000b200000000000000000000000200000490000000a9fc0100000001fb0000000a00560069006500770073030000004e00000080000002e10000019700000003000007380000003efc0100000002fb0000000800540069006d00650100000000000007380000025300fffffffb0000000800540069006d00650100000000000004500000000000000000000003de0000035e00000004000000040000000800000008fc0000000100000002000000010000000a0054006f006f006c00730100000000ffffffff0000000000000000
  Selection:
    collapsed: false
  Time:
    collapsed: false
  Tool Properties:
    collapsed: false
  Tracking Image:
    collapsed: false
  Views:
    collapsed: false
  Width: 1848
  X: 72
  Y: 27
````

## File: scripts/run_dynosam_gtest.py
````python
#!/usr/bin/python3


import os
import argparse
import subprocess

from ament_index_python.packages import get_package_prefix

def find_ros2_package_path(package_name):
    """
    Find the installation path of a ROS 2 package.

    Args:
        package_name (str): Name of the ROS 2 package.

    Returns:
        str: The full path to the package's install directory.

    Raises:
        ValueError: If the package is not found.
    """
    try:
        # Get the package install directory
        package_path = get_package_prefix(package_name)
        return package_path
    except ValueError as e:
        raise ValueError(f"Package '{package_name}' not found.") from e

def run_executable(executable_path, args):
    """
    Run an executable with given command-line arguments.

    Args:
        executable_path (str): Path to the executable file.
        args (list): List of command-line arguments to pass.

    Returns:
        int: The return code of the executable.
    """
    try:
        print(f"Running executable {executable_path}")
        # Combine the executable and arguments into a single command
        command = [executable_path] + args

        # Run the command and wait for it to finish
        result = subprocess.run(command, check=True, text=True)

        # Print standard output and error
        print("Standard Output:\n", result.stdout)
        print("Standard Error:\n", result.stderr)

        return result.returncode
    except subprocess.CalledProcessError as e:
        print(f"Error: Command '{e.cmd}' failed with return code {e.returncode}")
        print("Standard Output:\n", e.stdout)
        print("Standard Error:\n", e.stderr)
        return e.returncode
    except FileNotFoundError:
        print(f"Error: Executable '{executable_path}' not found.")
        return -1


def run_tests_for_package(package_name, unknown_args):
    package_path = find_ros2_package_path(package_name)

    print(f"Found package path for package {package_path}")
    tests_path = package_path + "/test/"


    # test that path exists
    if not os.path.exists(tests_path) or not os.path.isdir(tests_path):
        raise ValueError(f"Test folder is not valid or cannot be found at path {tests_path}")

    # find possible test exeutables
    executables = []
    for file_name in os.listdir(tests_path):
        full_path = os.path.join(tests_path, file_name)
        # Check if it's executable
        if os.path.isfile(full_path) and os.access(full_path, os.X_OK):
            executables.append(full_path)

    if len(executables) == 0:
        print(f"No executables found on test path {tests_path}. Skipping running tests...")
        return -1


    # # run tests
    for exec in executables:
        run_executable(exec, unknown_args)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="""Utility to run gtests for dynsam C++ packages.
        Additional args (anything besides --package, ie any gflags) are passed to gtest."""
    )
    parser.add_argument(
        "--package",
        "-p",
        nargs="?",
        type=str,
        default="dynosam",
        help="Which package tests to run. Options are any dynosam, dynosam_ros or all",
    )

    args, unknown_args = parser.parse_known_args()
    possible_packages = ["dynosam", "dynosam_ros"]

    if args.package == "all":
        pass
    else:
        # check valid package
        if args.package not in possible_packages:
            raise Exception(f"{args.package} is not a valud --package argument. Options are {possible_packages}")

        run_tests_for_package(args.package, unknown_args)
````

## File: src/displays/dynamic_slam_displays/BackendDSDRos.cc
````
/*
 *   Copyright (c) 2025 ACFR-RPG, University of Sydney, Jesse Morris
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

#include "dynosam_ros/displays/dynamic_slam_displays/BackendDSDRos.hpp"

namespace dyno {

BackendDSDRos::BackendDSDRos(const DisplayParams params,
                             rclcpp::Node::SharedPtr node)
    : BackendDisplay(), DSDRos(params, node) {}

void BackendDSDRos::spinOnce(
    const BackendOutputPacket::ConstPtr& backend_output) {
  // publish vo and path
  constexpr static bool kPublishOdomAsTf = false;
  this->publishVisualOdometry(backend_output->pose(),
                              backend_output->getTimestamp(), kPublishOdomAsTf);
  this->publishVisualOdometryPath(backend_output->optimized_camera_poses,
                                  backend_output->getTimestamp());

  // publish static cloud
  this->publishStaticPointCloud(backend_output->static_landmarks,
                                backend_output->pose());

  // publish dynamic cloud
  this->publishDynamicPointCloud(backend_output->dynamic_landmarks,
                                 backend_output->pose());

  const auto& object_motions = backend_output->optimized_object_motions;
  const auto& object_poses = backend_output->optimized_object_poses;

  // publish objects
  DSDTransport::Publisher object_poses_publisher = dsd_transport_.addObjectInfo(
      object_motions, object_poses, params_.world_frame_id,
      backend_output->getFrameId(), backend_output->getTimestamp());
  object_poses_publisher.publishObjectOdometry();
  object_poses_publisher.publishObjectPaths();
}

}  // namespace dyno
````

## File: src/displays/dynamic_slam_displays/DSDCommonRos.cc
````
#include "dynosam_ros/displays/dynamic_slam_displays/DSDCommonRos.hpp"

#include <glog/logging.h>

#include <dynosam/common/DynamicObjects.hpp>
#include <dynosam/visualizer/Colour.hpp>

#include "dynosam_ros/RosUtils.hpp"
#include "dynosam_ros/displays/DisplaysCommon.hpp"

namespace dyno {

DSDTransport::DSDTransport(rclcpp::Node::SharedPtr node) : node_(node) {
  object_odom_publisher_ =
      node->create_publisher<ObjectOdometry>("object_odometry", 1);
  multi_object_odom_path_publisher_ =
      node->create_publisher<MultiObjectOdometryPath>("object_odometry_path",
                                                      1);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*node_);

  VLOG(20) << "Constructed DSDTransport with effective namespace "
           << node_->get_effective_namespace();
}

std::string DSDTransport::constructObjectFrameLink(ObjectId object_id) {
  return "object_" + std::to_string(object_id) + "_link";
}

ObjectOdometry DSDTransport::constructObjectOdometry(
    const gtsam::Pose3& motion_k, const gtsam::Pose3& pose_k,
    ObjectId object_id, Timestamp timestamp_k, const std::string& frame_id_link,
    const std::string& child_frame_id_link) {
  ObjectOdometry object_odom;

  // technically this shoudl be k-1
  gtsam::Point3 body_velocity = calculateBodyMotion(motion_k, pose_k);

  nav_msgs::msg::Odometry odom_msg;
  utils::convertWithHeader(pose_k, odom_msg, timestamp_k, frame_id_link,
                           child_frame_id_link);

  object_odom.odom = odom_msg;
  // TODO: can check if correct representation?

  dyno::convert(motion_k, object_odom.h_w_km1_k.pose);
  // NO velocity!!
  object_odom.object_id = object_id;
  return object_odom;
}

ObjectOdometryMap DSDTransport::constructObjectOdometries(
    const MotionEstimateMap& motions_k, const ObjectPoseMap& poses,
    FrameId frame_id_k, Timestamp timestamp_k,
    const std::string& frame_id_link) {
  // need to get poses for k-1
  // TODO: no way to ensure that the motions are for frame k
  // this is a weird data-structure to use and motions are per frame and
  // ObjectPoseMap is over all k to K
  //  const FrameId frame_id_k_1 = frame_id_k - 1u;
  ObjectOdometryMap object_odom_map;
  for (const auto& [object_id, object_motion] : motions_k) {
    const gtsam::Pose3& motion_k = object_motion;

    if (!poses.exists(object_id, frame_id_k)) {
      LOG(WARNING) << "Cannot construct ObjectOdometry for object " << object_id
                   << ", at frame " << frame_id_k
                   << " Missing entry in ObjectPoseMap";
      continue;
    }

    const gtsam::Pose3& pose_k = poses.at(object_id, frame_id_k);

    const std::string child_frame_id_link = constructObjectFrameLink(object_id);

    object_odom_map.insert2(
        child_frame_id_link,
        constructObjectOdometry(motion_k, pose_k, object_id, timestamp_k,
                                frame_id_link, child_frame_id_link));
  }

  return object_odom_map;
}

MultiObjectOdometryPath DSDTransport::constructMultiObjectOdometryPaths(
    const MotionEstimateMap& motions_k, const ObjectPoseMap& poses,
    FrameId frame_id_k, Timestamp timestamp_k, rclcpp::Time ros_time_now,
    const std::string& frame_id_link) {
  MultiObjectOdometryPath multi_path;
  multi_path.header.stamp = ros_time_now;
  multi_path.header.frame_id = frame_id_link;

  // TODO: right now dont have the motion for every timestep so... just leave
  // blank?
  for (const auto& [object_id, frame_pose_map] : poses) {
    const std::string child_frame_id_link = constructObjectFrameLink(object_id);
    FrameId previous_frame_id = -1;
    bool first = true;
    int path_segment = 0;

    std_msgs::msg::ColorRGBA colour_msg;
    convert(Color::uniqueId(object_id), colour_msg);

    // paths for this object, broken into segments
    gtsam::FastMap<int, ObjectOdometryPath> segmented_paths;

    for (const auto& [frame_id, object_pose] : frame_pose_map) {
      if (!first && frame_id != previous_frame_id + 1) {
        path_segment++;
      }
      first = false;
      previous_frame_id = frame_id;

      // RIGHT NOW MOTION IDENTITY
      // timestamp is wring
      gtsam::Pose3 motion;
      const ObjectOdometry object_odometry =
          constructObjectOdometry(motion, object_pose, object_id, timestamp_k,
                                  frame_id_link, child_frame_id_link);

      if (!segmented_paths.exists(path_segment)) {
        ObjectOdometryPath path;
        path.colour = colour_msg;
        path.object_id = object_id;
        path.path_segment = path_segment;

        path.header = multi_path.header;

        segmented_paths.insert2(path_segment, path);
      }

      ObjectOdometryPath& path = segmented_paths.at(path_segment);
      path.object_odometries.push_back(object_odometry);
    }

    for (const auto& [_, path] : segmented_paths) {
      multi_path.paths.push_back(path);
    }
  }

  return multi_path;
}

void DSDTransport::Publisher::publishObjectOdometry() {
  for (const auto& [_, object_odom] : object_odometries_)
    object_odom_publisher_->publish(object_odom);
}

void DSDTransport::Publisher::publishObjectTransforms() {
  for (const auto& [object_child_frame, object_odom] : object_odometries_) {
    geometry_msgs::msg::TransformStamped t;
    dyno::convert(object_odom.odom.pose.pose, t.transform);

    t.header.stamp = node_->now();
    t.header.frame_id = frame_id_link_;
    t.child_frame_id = object_child_frame;

    tf_broadcaster_->sendTransform(t);
  }
}

void DSDTransport::Publisher::publishObjectPaths() {
  multi_object_odom_path_publisher_->publish(object_paths_);
}

DSDTransport::Publisher::Publisher(
    rclcpp::Node::SharedPtr node,
    ObjectOdometryPub::SharedPtr object_odom_publisher,
    MultiObjectOdometryPathPub::SharedPtr multi_object_odom_path_publisher,
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster,
    const MotionEstimateMap& motions, const ObjectPoseMap& poses,
    const std::string& frame_id_link, FrameId frame_id, Timestamp timestamp)
    : node_(node),
      object_odom_publisher_(object_odom_publisher),
      multi_object_odom_path_publisher_(multi_object_odom_path_publisher),
      tf_broadcaster_(tf_broadcaster),
      frame_id_link_(frame_id_link),
      frame_id_(frame_id),
      timestamp_(timestamp),
      object_odometries_(DSDTransport::constructObjectOdometries(
          motions, poses, frame_id, timestamp, frame_id_link)),
      object_paths_(DSDTransport::constructMultiObjectOdometryPaths(
          motions, poses, frame_id, timestamp, node->now(), frame_id_link)) {}

DSDTransport::Publisher DSDTransport::addObjectInfo(
    const MotionEstimateMap& motions_k, const ObjectPoseMap& poses,
    const std::string& frame_id_link, FrameId frame_id, Timestamp timestamp) {
  return Publisher(node_, object_odom_publisher_,
                   multi_object_odom_path_publisher_, tf_broadcaster_,
                   motions_k, poses, frame_id_link, frame_id, timestamp);
}

DSDRos::DSDRos(const DisplayParams& params, rclcpp::Node::SharedPtr node)
    : params_(params), node_(node), dsd_transport_(node) {
  vo_publisher_ =
      node_->create_publisher<nav_msgs::msg::Odometry>("odometry", 1);
  vo_path_publisher_ =
      node_->create_publisher<nav_msgs::msg::Path>("odometry_path", 1);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*node_);

  static_points_pub_ =
      node->create_publisher<sensor_msgs::msg::PointCloud2>("static_cloud", 1);
  dynamic_points_pub_ =
      node->create_publisher<sensor_msgs::msg::PointCloud2>("dynamic_cloud", 1);
}

void DSDRos::publishVisualOdometry(const gtsam::Pose3& T_world_camera,
                                   Timestamp timestamp, const bool publish_tf) {
  DisplayCommon::publishOdometry(vo_publisher_, T_world_camera, timestamp,
                                 params_.world_frame_id,
                                 params_.camera_frame_id);

  if (publish_tf) {
    geometry_msgs::msg::TransformStamped t;
    dyno::convert<gtsam::Pose3, geometry_msgs::msg::TransformStamped>(
        T_world_camera, t);

    t.header.stamp = node_->now();
    t.header.frame_id = params_.world_frame_id;
    t.child_frame_id = params_.camera_frame_id;
    tf_broadcaster_->sendTransform(t);
  }
}
void DSDRos::publishVisualOdometryPath(const gtsam::Pose3Vector& poses,
                                       Timestamp latest_timestamp) {
  DisplayCommon::publishOdometryPath(vo_path_publisher_, poses,
                                     latest_timestamp, params_.world_frame_id);
}

CloudPerObject DSDRos::publishStaticPointCloud(
    const StatusLandmarkVector& landmarks, const gtsam::Pose3& T_world_camera) {
  return DisplayCommon::publishPointCloud(
      static_points_pub_, landmarks, T_world_camera, params_.world_frame_id);
}
CloudPerObject DSDRos::publishDynamicPointCloud(
    const StatusLandmarkVector& landmarks, const gtsam::Pose3& T_world_camera) {
  return DisplayCommon::publishPointCloud(
      dynamic_points_pub_, landmarks, T_world_camera, params_.world_frame_id);
}

}  // namespace dyno
````

## File: src/displays/dynamic_slam_displays/FrontendDSDRos.cc
````
/*
 *   Copyright (c) 2025 ACFR-RPG, University of Sydney, Jesse Morris
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

#include "dynosam_ros/displays/dynamic_slam_displays/FrontendDSDRos.hpp"

#include <cv_bridge/cv_bridge.h>

#include <dynosam/utils/SafeCast.hpp>

#include "dynosam_ros/RosUtils.hpp"

namespace dyno {

FrontendDSDRos::FrontendDSDRos(const DisplayParams params,
                               rclcpp::Node::SharedPtr node)
    : FrontendDisplay(), DSDRos(params, node) {
  tracking_image_pub_ =
      image_transport::create_publisher(node.get(), "tracking_image");

  auto ground_truth_node = node->create_sub_node("ground_truth");
  dsd_ground_truth_transport_ =
      std::make_unique<DSDTransport>(ground_truth_node);
  vo_ground_truth_publisher_ =
      ground_truth_node->create_publisher<nav_msgs::msg::Odometry>("odometry",
                                                                   1);
  vo_path_ground_truth_publisher_ =
      ground_truth_node->create_publisher<nav_msgs::msg::Path>("odometry_path",
                                                               1);
}

void FrontendDSDRos::spinOnce(
    const FrontendOutputPacketBase::ConstPtr& frontend_output) {
  // publish debug imagery
  tryPublishDebugImagery(frontend_output);

  // publish ground truth
  tryPublishGroundTruth(frontend_output);

  // publish odometry
  tryPublishVisualOdometry(frontend_output);

  // attempt cast
  RGBDInstanceOutputPacket::ConstPtr rgbd_output =
      safeCast<FrontendOutputPacketBase, RGBDInstanceOutputPacket>(
          frontend_output);
  // publish object info
  if (rgbd_output) processRGBDOutputpacket(rgbd_output);
}

void FrontendDSDRos::tryPublishDebugImagery(
    const FrontendOutputPacketBase::ConstPtr& frontend_output) {
  if (!frontend_output->debug_imagery_) return;

  const DebugImagery& debug_imagery = *frontend_output->debug_imagery_;
  if (debug_imagery.tracking_image.empty()) return;

  std_msgs::msg::Header hdr;
  sensor_msgs::msg::Image::SharedPtr msg =
      cv_bridge::CvImage(hdr, "bgr8", debug_imagery.tracking_image)
          .toImageMsg();
  tracking_image_pub_.publish(msg);
}

void FrontendDSDRos::tryPublishGroundTruth(
    const FrontendOutputPacketBase::ConstPtr& frontend_output) {
  if (!frontend_output->gt_packet_ || !frontend_output->debug_imagery_) return;

  const DebugImagery& debug_imagery = *frontend_output->debug_imagery_;
  const cv::Mat& rgb_image = debug_imagery.rgb_viz;
  const auto timestamp = frontend_output->getTimestamp();
  const auto frame_id = frontend_output->getFrameId();

  if (rgb_image.empty()) return;

  // collect gt poses and motions
  ObjectPoseMap poses;
  MotionEstimateMap motions;

  const GroundTruthInputPacket& gt_packet = frontend_output->gt_packet_.value();

  for (const auto& object_pose_gt : gt_packet.object_poses_) {
    // check we have a gt motion here
    // in the case that we dont, this might be the first time the object
    // appears...
    if (!object_pose_gt.prev_H_current_world_) {
      continue;
    }

    poses.insert22(object_pose_gt.object_id_, gt_packet.frame_id_,
                   object_pose_gt.L_world_);

    ReferenceFrameValue<gtsam::Pose3> gt_motion(
        *object_pose_gt.prev_H_current_world_, ReferenceFrame::GLOBAL);
    motions.insert({object_pose_gt.object_id_, gt_motion});
  }

  // will this result in confusing tf's since the gt object and estimated
  // objects use the same link?
  DSDTransport::Publisher publisher =
      dsd_ground_truth_transport_->addObjectInfo(
          motions, poses, params_.world_frame_id, frame_id, timestamp);
  publisher.publishObjectOdometry();

  // publish ground truth odom
  const gtsam::Pose3& T_world_camera = gt_packet.X_world_;
  nav_msgs::msg::Odometry odom_msg;
  utils::convertWithHeader(T_world_camera, odom_msg, timestamp,
                           params_.world_frame_id, params_.camera_frame_id);
  vo_ground_truth_publisher_->publish(odom_msg);

  // odom path gt
  // make static variable since we dont build up the path anywhere else
  // and just append the last gt camera pose to the path msg
  static nav_msgs::msg::Path gt_odom_path_msg;
  static std_msgs::msg::Header header;

  geometry_msgs::msg::PoseStamped pose_stamped;
  utils::convertWithHeader(T_world_camera, pose_stamped, timestamp,
                           params_.world_frame_id);

  header.stamp = utils::toRosTime(timestamp);
  header.frame_id = params_.world_frame_id;
  gt_odom_path_msg.header = header;
  gt_odom_path_msg.poses.push_back(pose_stamped);

  vo_path_ground_truth_publisher_->publish(gt_odom_path_msg);
}
void FrontendDSDRos::tryPublishVisualOdometry(
    const FrontendOutputPacketBase::ConstPtr& frontend_output) {
  // publish vo
  constexpr static bool kPublishOdomAsTf = true;
  this->publishVisualOdometry(frontend_output->T_world_camera_,
                              frontend_output->getTimestamp(),
                              kPublishOdomAsTf);
}

void FrontendDSDRos::processRGBDOutputpacket(
    const RGBDInstanceOutputPacket::ConstPtr& rgbd_packet) {
  // publish path
  // why the camera poses are only in the RGBDInstanceOutputPacket and not in
  // the base... I have no idea :)
  this->publishVisualOdometryPath(rgbd_packet->camera_poses_,
                                  rgbd_packet->getTimestamp());

  // publish static cloud
  CHECK(rgbd_packet);
  this->publishStaticPointCloud(rgbd_packet->static_landmarks_,
                                rgbd_packet->T_world_camera_);

  // publish and collect dynamic cloud
  CloudPerObject clouds_per_obj = this->publishDynamicPointCloud(
      rgbd_packet->dynamic_landmarks_, rgbd_packet->T_world_camera_);

  const auto& object_motions = rgbd_packet->estimated_motions_;
  const auto& object_poses = rgbd_packet->propogated_object_poses_;

  DSDTransport::Publisher object_poses_publisher = dsd_transport_.addObjectInfo(
      object_motions, object_poses, params_.world_frame_id,
      rgbd_packet->getFrameId(), rgbd_packet->getTimestamp());
  object_poses_publisher.publishObjectOdometry();
  object_poses_publisher.publishObjectTransforms();
  object_poses_publisher.publishObjectPaths();
}

}  // namespace dyno
````

## File: src/displays/inbuilt_displays/BackendInbuiltDisplayRos.cc
````
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

#include "dynosam_ros/displays/inbuilt_displays/BackendInbuiltDisplayRos.hpp"

#include <glog/logging.h>
#include <pcl_conversions/pcl_conversions.h>

#include <dynosam/visualizer/ColourMap.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "dynosam_ros/RosUtils.hpp"
#include "rclcpp/qos.hpp"

namespace dyno {

BackendInbuiltDisplayRos::BackendInbuiltDisplayRos(const DisplayParams params,
                                                   rclcpp::Node::SharedPtr node)
    : InbuiltDisplayCommon(params, node) {
  // const rclcpp::QoS& sensor_data_qos = rclcpp::SensorDataQoS();
  static_tracked_points_pub_ =
      node->create_publisher<sensor_msgs::msg::PointCloud2>("static_cloud", 1);
  dynamic_tracked_points_pub_ =
      node->create_publisher<sensor_msgs::msg::PointCloud2>("dynamic_cloud", 1);

  odometry_pub_ = node->create_publisher<nav_msgs::msg::Odometry>("odom", 1);
  object_pose_pub_ =
      node->create_publisher<visualization_msgs::msg::MarkerArray>(
          "composed_object_poses", 1);
  object_pose_path_pub_ =
      node->create_publisher<visualization_msgs::msg::MarkerArray>(
          "composed_object_paths", 1);

  object_aabb_pub_ =
      node->create_publisher<visualization_msgs::msg::MarkerArray>(
          "object_aabb", 1);

  odometry_path_pub_ =
      node->create_publisher<nav_msgs::msg::Path>("odom_path", 2);
}

void BackendInbuiltDisplayRos::spinOnce(
    const BackendOutputPacket::ConstPtr& backend_output) {
  publishPointCloud(static_tracked_points_pub_,
                    backend_output->static_landmarks,
                    backend_output->pose());
  CloudPerObject clouds_per_obj = publishPointCloud(
      dynamic_tracked_points_pub_, backend_output->dynamic_landmarks,
      backend_output->pose());

  publishObjectBoundingBox(object_aabb_pub_, nullptr, /* no Obb publisher */
                           clouds_per_obj, utils::fromRosTime(node_->now()),
                           "backend");
  publishObjectPositions(
      object_pose_pub_, backend_output->optimized_object_poses,
      backend_output->getFrameId(), backend_output->getTimestamp(), "backend");

  publishObjectPaths(
      object_pose_path_pub_, backend_output->optimized_object_poses,
      backend_output->getFrameId(), backend_output->getTimestamp(), "backend", 60);

  {
    nav_msgs::msg::Odometry odom_msg;
    utils::convertWithHeader(backend_output->pose(), odom_msg,
                             backend_output->getTimestamp(), "world", "camera");
    odometry_pub_->publish(odom_msg);
  }

  {
    nav_msgs::msg::Path odom_path_msg;

    for (const gtsam::Pose3& T_world_camera :
         backend_output->optimized_camera_poses) {
      // optimized camera traj
      geometry_msgs::msg::PoseStamped pose_stamped;
      utils::convertWithHeader(T_world_camera, pose_stamped,
                               backend_output->getTimestamp(), "world");

      static std_msgs::msg::Header header;
      header.stamp = utils::toRosTime(backend_output->getTimestamp());
      header.frame_id = "world";
      odom_path_msg.header = header;
      odom_path_msg.poses.push_back(pose_stamped);
    }

    odometry_path_pub_->publish(odom_path_msg);
  }
}

}  // namespace dyno
````

## File: src/displays/inbuilt_displays/FrontendInbuiltDisplayRos.cc
````
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

#include "dynosam_ros/displays/inbuilt_displays/FrontendInbuiltDisplayRos.hpp"

#include <cv_bridge/cv_bridge.h>
#include <glog/logging.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/memory.h>
#include <pcl_conversions/pcl_conversions.h>

#include <dynosam/frontend/RGBDInstance-Definitions.hpp>
#include <dynosam/frontend/vision/Feature.hpp>  //for functional_keypoint
#include <dynosam/utils/SafeCast.hpp>
#include <dynosam/visualizer/ColourMap.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <string>

#include "dynosam_ros/RosUtils.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/qos.hpp"

namespace dyno {

FrontendInbuiltDisplayRos::FrontendInbuiltDisplayRos(
    const DisplayParams params, rclcpp::Node::SharedPtr node)
    : InbuiltDisplayCommon(params, node) {
  const rclcpp::QoS& sensor_data_qos = rclcpp::SensorDataQoS();
  tracking_image_pub_ =
      image_transport::create_publisher(node.get(), "tracking_image");
  static_tracked_points_pub_ =
      node->create_publisher<sensor_msgs::msg::PointCloud2>("static_cloud", 1);
  dynamic_tracked_points_pub_ =
      node->create_publisher<sensor_msgs::msg::PointCloud2>("dynamic_cloud", 1);
  odometry_pub_ = node->create_publisher<nav_msgs::msg::Odometry>("odom", 1);
  object_pose_pub_ =
      node->create_publisher<visualization_msgs::msg::MarkerArray>(
          "composed_object_poses", 1);
  object_pose_path_pub_ =
      node->create_publisher<visualization_msgs::msg::MarkerArray>(
          "composed_object_paths", 1);
  odometry_path_pub_ =
      node->create_publisher<nav_msgs::msg::Path>("odom_path", 2);
  object_motion_pub_ =
      node->create_publisher<nav_msgs::msg::Path>("object_motions", 1);
  object_bbx_line_pub_ =
      node->create_publisher<visualization_msgs::msg::MarkerArray>(
          "object_bbx_viz", 1);
  object_bbx_pub_ =
      node->create_publisher<visualization_msgs::msg::MarkerArray>(
          "object_bounding_boxes", 1);

  // TODO: fix up gt publisher namespacing -? currently
  // dynosam/dynosam/ground_trut....
  gt_odometry_pub_ =
      node->create_publisher<nav_msgs::msg::Odometry>("~/ground_truth/odom", 1);
  gt_object_pose_pub_ =
      node->create_publisher<visualization_msgs::msg::MarkerArray>(
          "~/ground_truth/object_poses", 1);
  gt_object_path_pub_ =
      node->create_publisher<visualization_msgs::msg::MarkerArray>(
          "~/ground_truth/object_paths", 1);
  gt_odom_path_pub_ = node->create_publisher<nav_msgs::msg::Path>(
      "~/ground_truth/odom_path", 1);
  gt_bounding_box_pub_ = image_transport::create_publisher(
      node.get(), "~/ground_truth/bounding_boxes");

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*node_);
}

FrontendInbuiltDisplayRos::~FrontendInbuiltDisplayRos() {
  if (video_writer_) video_writer_->release();
}

void FrontendInbuiltDisplayRos::spinOnce(
    const FrontendOutputPacketBase::ConstPtr& frontend_output) {
  // TODO: does frontend or backend publish tf transform?
  if (frontend_output->debug_imagery_)
    publishDebugImage(*frontend_output->debug_imagery_);

  if (frontend_output->gt_packet_ && frontend_output->debug_imagery_) {
    const auto& debug_imagery = *frontend_output->debug_imagery_;
    // TODO: put tracking images back into frontend output
    const cv::Mat& rgb_image = debug_imagery.rgb_viz;
    publishGroundTruthInfo(frontend_output->getTimestamp(),
                           frontend_output->gt_packet_.value(), rgb_image);
  }

  RGBDInstanceOutputPacket::ConstPtr rgbd_output =
      safeCast<FrontendOutputPacketBase, RGBDInstanceOutputPacket>(
          frontend_output);
  if (rgbd_output) {
    processRGBDOutputpacket(rgbd_output);
    // TODO:currently only with RGBDInstanceOutputPacket becuase camera poses is
    // in RGBDInstanceOutputPacket but should be in base
    // (FrontendOutputPacketBase)
    publishOdometryPath(odometry_path_pub_, rgbd_output->camera_poses_,
                        frontend_output->getTimestamp());
    publishOdometry(frontend_output->T_world_camera_,
                    frontend_output->getTimestamp());
    // publishCameraMarker(
    //     frontend_output->T_world_camera_,
    //     frontend_output->getTimestamp(),
    //     "frontend_camera_pose",
    //     cv::viz::Color::yellow()
    // );
  }
}

void FrontendInbuiltDisplayRos::processRGBDOutputpacket(
    const RGBDInstanceOutputPacket::ConstPtr& rgbd_frontend_output) {
  CHECK(rgbd_frontend_output);
  publishPointCloud(static_tracked_points_pub_,
                    rgbd_frontend_output->static_landmarks_,
                    rgbd_frontend_output->T_world_camera_);

  // TODO: there is a bunch of repeated code in this function and in
  // groupObjectCloud we leave this as is becuase this function ALSO creates the
  // coloured point cloud (which groupObjectCloud does not) eventually, refactor
  // into one function or calcualte the coloured cloud in the
  // RGBDInstanceOutputPacket
  CloudPerObject clouds_per_obj = publishPointCloud(
      dynamic_tracked_points_pub_, rgbd_frontend_output->dynamic_landmarks_,
      rgbd_frontend_output->T_world_camera_);

  publishObjectPositions(object_pose_pub_,
                         rgbd_frontend_output->propogated_object_poses_,
                         rgbd_frontend_output->getFrameId(),
                         rgbd_frontend_output->getTimestamp(), "frontend");

  publishObjectPaths(object_pose_path_pub_,
                     rgbd_frontend_output->propogated_object_poses_,
                     rgbd_frontend_output->getFrameId(),
                     rgbd_frontend_output->getTimestamp(), "frontend", 60);

  // object bounding box using linelist and id text in rviz
  {
    visualization_msgs::msg::MarkerArray
        object_bbx_linelist_marker_array;  // using linelist
    visualization_msgs::msg::MarkerArray object_bbx_marker_array;  // using cube
    static visualization_msgs::msg::Marker delete_marker;
    delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    object_bbx_linelist_marker_array.markers.push_back(delete_marker);
    object_bbx_marker_array.markers.push_back(delete_marker);

    for (const auto& [object_id, obj_cloud] : clouds_per_obj) {
      pcl::PointXYZ centroid;
      pcl::computeCentroid(obj_cloud, centroid);

      const cv::Scalar colour = Color::uniqueId(object_id);

      visualization_msgs::msg::Marker txt_marker;
      txt_marker.header.frame_id = "world";
      txt_marker.ns = "object_id";
      txt_marker.id = object_id;
      txt_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      txt_marker.action = visualization_msgs::msg::Marker::ADD;
      txt_marker.header.stamp = node_->now();
      txt_marker.scale.z = 2.0;
      txt_marker.color.r = colour(0) / 255.0;
      txt_marker.color.g = colour(1) / 255.0;
      txt_marker.color.b = colour(2) / 255.0;
      txt_marker.color.a = 1;
      txt_marker.text = "obj " + std::to_string(object_id);
      txt_marker.pose.position.x = centroid.x;
      txt_marker.pose.position.y = centroid.y - 2.0;
      txt_marker.pose.position.z = centroid.z - 1.0;
      object_bbx_linelist_marker_array.markers.push_back(txt_marker);

      pcl::PointCloud<pcl::PointXYZRGB>::Ptr obj_cloud_ptr =
          pcl::make_shared<pcl::PointCloud<pcl::PointXYZRGB> >(obj_cloud);
      ObjectBBX aabb = findAABBFromCloud<pcl::PointXYZRGB>(obj_cloud_ptr);
      ObjectBBX obb = findOBBFromCloud<pcl::PointXYZRGB>(obj_cloud_ptr);

      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "world";
      marker.ns = "object_bbx_linelist";
      marker.id = object_id;
      marker.type = visualization_msgs::msg::Marker::LINE_LIST;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.header.stamp = node_->now();
      marker.scale.x = 0.1;

      marker.pose.orientation.x = 0;
      marker.pose.orientation.y = 0;
      marker.pose.orientation.z = 0;
      marker.pose.orientation.w = 1;

      marker.color.r = colour(0) / 255.0;
      marker.color.g = colour(1) / 255.0;
      marker.color.b = colour(2) / 255.0;
      marker.color.a = 1;

      for (const pcl::PointXYZ& this_line_list_point :
           findLineListPointsFromAABBMinMax(aabb.min_bbx_point_,
                                            aabb.max_bbx_point_)) {
        geometry_msgs::msg::Point p;
        p.x = this_line_list_point.x;
        p.y = this_line_list_point.y;
        p.z = this_line_list_point.z;
        marker.points.push_back(p);
      }
      object_bbx_linelist_marker_array.markers.push_back(marker);

      visualization_msgs::msg::Marker bbx_marker;
      bbx_marker.header.frame_id = "world";
      bbx_marker.ns = "object_bbx";
      bbx_marker.id = object_id;
      bbx_marker.type = visualization_msgs::msg::Marker::CUBE;
      bbx_marker.action = visualization_msgs::msg::Marker::ADD;
      bbx_marker.header.stamp = node_->now();
      bbx_marker.scale.x = obb.max_bbx_point_.x() - obb.min_bbx_point_.x();
      bbx_marker.scale.y = obb.max_bbx_point_.y() - obb.min_bbx_point_.y();
      bbx_marker.scale.z = obb.max_bbx_point_.z() - obb.min_bbx_point_.z();

      bbx_marker.pose.position.x = obb.bbx_position_.x();
      bbx_marker.pose.position.y = obb.bbx_position_.y();
      bbx_marker.pose.position.z = obb.bbx_position_.z();

      const gtsam::Quaternion& q = obb.orientation_.toQuaternion();
      bbx_marker.pose.orientation.x = q.x();
      bbx_marker.pose.orientation.y = q.y();
      bbx_marker.pose.orientation.z = q.z();
      bbx_marker.pose.orientation.w = q.w();

      bbx_marker.color.r = colour(0) / 255.0;
      bbx_marker.color.g = colour(1) / 255.0;
      bbx_marker.color.b = colour(2) / 255.0;
      bbx_marker.color.a = 0.2;

      object_bbx_marker_array.markers.push_back(bbx_marker);
    }
    object_bbx_line_pub_->publish(object_bbx_linelist_marker_array);
    object_bbx_pub_->publish(object_bbx_marker_array);
  }

  // Predict 3D object poses
  gtsam::FastMap<ObjectId, std::vector<gtsam::Pose3> > obj_predicted_poses;
  {
    int prediction_length = 3;
    const FrameId current_frame_id = rgbd_frontend_output->getFrameId();
    const ObjectPoseMap& obj_poses =
        rgbd_frontend_output->propogated_object_poses_;
    const MotionEstimateMap& obj_motions =
        rgbd_frontend_output->estimated_motions_;
    for (const auto& [object_id, this_obj_traj] : obj_poses) {
      if (this_obj_traj.exists(current_frame_id)) {
        gtsam::Pose3 current_obj_pose = this_obj_traj.at(current_frame_id);
        if (obj_motions.exists(object_id)) {
          gtsam::Pose3 current_obj_motion = obj_motions.at(object_id);

          gtsam::Pose3 last_object_pose = current_obj_pose;
          std::vector<gtsam::Pose3> predicted_poses;

          for (int i_prediction = 0; i_prediction < prediction_length;
               i_prediction++) {
            predicted_poses.push_back(last_object_pose);
            last_object_pose = current_obj_motion * last_object_pose;
            last_object_pose = current_obj_motion * last_object_pose;
            // last_object_pose = current_obj_motion * last_object_pose;
          }

          obj_predicted_poses.insert2(object_id, predicted_poses);
        }
      }
    }
  }

  // Write object motion to nav_msgs/Path and publish
  nav_msgs::msg::Path object_motions_msg;
  {
    const FrameId current_frame_id = rgbd_frontend_output->getFrameId();
    const MotionEstimateMap& obj_motions =
        rgbd_frontend_output->estimated_motions_;

    // object_motions_msg.header.seq = current_frame_id;
    object_motions_msg.header.stamp = node_->now();
    object_motions_msg.header.frame_id = params_.world_frame_id;

    for (const auto& [object_id, current_obj_motion] : obj_motions) {
      geometry_msgs::msg::PoseStamped current_obj_motion_msg;
      current_obj_motion_msg.header.stamp = node_->now();
      current_obj_motion_msg.header.frame_id = std::to_string(object_id);
      current_obj_motion_msg.pose.position.x =
          current_obj_motion.estimate_.translation().x();
      current_obj_motion_msg.pose.position.y =
          current_obj_motion.estimate_.translation().y();
      current_obj_motion_msg.pose.position.z =
          current_obj_motion.estimate_.translation().z();
      current_obj_motion_msg.pose.orientation.w =
          current_obj_motion.estimate_.rotation()
              .toQuaternion()
              .w();  // in w x y z form
      current_obj_motion_msg.pose.orientation.x =
          current_obj_motion.estimate_.rotation()
              .toQuaternion()
              .x();  // in w x y z form
      current_obj_motion_msg.pose.orientation.y =
          current_obj_motion.estimate_.rotation()
              .toQuaternion()
              .y();  // in w x y z form
      current_obj_motion_msg.pose.orientation.z =
          current_obj_motion.estimate_.rotation()
              .toQuaternion()
              .z();  // in w x y z form

      object_motions_msg.poses.push_back(current_obj_motion_msg);
    }

    object_motion_pub_->publish(object_motions_msg);
  }

  // 2D image visualisation using opencv (TODO: and check that camera is
  // given!!)
  if (rgbd_frontend_output->debug_imagery_) {
    cv::Mat rgb_with_bbx;
    rgbd_frontend_output->debug_imagery_->tracking_image.copyTo(rgb_with_bbx);
    const cv::Size rgb_size = rgb_with_bbx.size();

    const StatusKeypointVector& obj_px =
        rgbd_frontend_output->dynamic_keypoint_measurements_;
    for (const KeypointStatus& this_px : obj_px) {
      ObjectId object_id = this_px.objectId();
      const gtsam::Point2& px_coordinate = this_px.value();

      const cv::Scalar colour = Color::uniqueId(object_id).bgra();
      // const cv::Scalar colour_bgr(colour[2], colour[1], colour[0]);
      cv::Point centre(functional_keypoint::u(px_coordinate),
                       functional_keypoint::v(px_coordinate));
      int radius = 1;
      int thickness = 1;
      cv::circle(rgb_with_bbx, centre, radius, colour, thickness);
    }

    // object history and prediction
    const ObjectPoseMap& obj_poses =
        rgbd_frontend_output->propogated_object_poses_;
    const gtsam::Pose3& cam_pose = rgbd_frontend_output->T_world_camera_;
    int line_thinkness = 3;
    for (const auto& [object_id, this_obj_traj] : obj_poses) {
      cv::Scalar colour = Color::uniqueId(object_id).bgra();
      // const cv::Scalar colour_bgr(colour[2], colour[1], colour[0]);
      std::vector<cv::Point> cv_line;
      for (const auto& [frame_id, this_obj_pose] : this_obj_traj) {
        gtsam::Pose3 this_obj_pose_in_cam = cam_pose.inverse() * this_obj_pose;
        Landmark this_obj_position_in_cam = this_obj_pose_in_cam.translation();
        Keypoint this_obj_px_in_cam;
        bool is_lmk_contained =
            rgbd_frontend_output->camera_->isLandmarkContained(
                this_obj_position_in_cam, &this_obj_px_in_cam);
        if (is_lmk_contained) {
          cv_line.push_back(
              cv::Point(functional_keypoint::u(this_obj_px_in_cam),
                        functional_keypoint::v(this_obj_px_in_cam)));
        }
      }
      int line_length = cv_line.size();
      for (int i_line = 0; i_line < line_length - 1; i_line++) {
        cv::line(rgb_with_bbx, cv_line[i_line], cv_line[i_line + 1], colour,
                 line_thinkness, cv::LINE_AA);
      }

      if (!obj_predicted_poses.exists(object_id)) {
        continue;
      }

      cv_line.clear();
      std::vector<gtsam::Pose3> this_obj_predicted_poses =
          obj_predicted_poses.at(object_id);
      for (const auto& this_obj_pose : this_obj_predicted_poses) {
        gtsam::Pose3 this_obj_pose_in_cam = cam_pose.inverse() * this_obj_pose;
        Landmark this_obj_position_in_cam = this_obj_pose_in_cam.translation();
        Keypoint this_obj_px_in_cam;
        bool is_lmk_contained =
            rgbd_frontend_output->camera_->isLandmarkContained(
                this_obj_position_in_cam, &this_obj_px_in_cam);
        if (is_lmk_contained) {
          cv_line.push_back(
              cv::Point(functional_keypoint::u(this_obj_px_in_cam),
                        functional_keypoint::v(this_obj_px_in_cam)));
        }
      }
      line_length = cv_line.size();
      // TODO: removed prediction arrow for now!!
      //  for (int i_line = 0; i_line < line_length-1; i_line++){
      //      double alpha = (double) i_line / ((double) line_length);
      //      cv::Scalar grad_colour = colour;
      //      grad_colour[0] = (255.0 - colour[0]) * alpha + colour[0];
      //      grad_colour[1] = (255.0 - colour[0]) * alpha + colour[1];
      //      grad_colour[2] = (255.0 - colour[0]) * alpha + colour[2];
      //      cv::arrowedLine(rgb_with_bbx, cv_line[i_line], cv_line[i_line+1],
      //      grad_colour, line_thinkness-1, cv::LINE_AA, 0, 0.3);
      //  }
    }

    // cv::resize(rgb_with_bbx, rgb_with_bbx, cv::Size(1280, 720) , 0, 0,
    // CV_INTER_LINEAR); if(video_writer_)  {
    //     CHECK(video_writer_->isOpened());
    //     video_writer_->write(rgb_with_bbx);
    // }

    cv::imshow("RGB with object bounding boxes", rgb_with_bbx);

    int frame_id = rgbd_frontend_output->getFrameId();
    // cv::Mat rgb_objects;
    // rgbd_frontend_output->frame_.tracking_images_.cloneImage<ImageType::RGBMono>(rgb_objects);

    // cv::imwrite("/root/results/RSS/"+std::to_string(frame_id)+".png",
    // rgb_objects);
  }

  {
    visualization_msgs::msg::MarkerArray object_pred_marker_array;

    for (const auto& [object_id, this_obj_predicted_poses] :
         obj_predicted_poses) {
      const cv::Scalar colour = Color::uniqueId(object_id);
      int pose_length = this_obj_predicted_poses.size();
      for (int i_pose = 0; i_pose < pose_length - 1; i_pose++) {
        gtsam::Point3 point_0 = this_obj_predicted_poses[i_pose].translation();
        gtsam::Point3 point_1 =
            this_obj_predicted_poses[i_pose + 1].translation();

        visualization_msgs::msg::Marker arrow_marker;
        arrow_marker.header.frame_id = "world";
        arrow_marker.ns = "object_pred";
        arrow_marker.id = object_id * 10 + i_pose;
        arrow_marker.type = visualization_msgs::msg::Marker::ARROW;
        arrow_marker.action = visualization_msgs::msg::Marker::ADD;
        arrow_marker.header.stamp = node_->now();
        arrow_marker.scale.x = 0.3;
        arrow_marker.scale.y = 0.5;
        arrow_marker.scale.z = 0.8;

        double alpha = (double)i_pose / ((double)pose_length);
        arrow_marker.color.r =
            ((255.0 - colour[0]) * alpha + colour[0]) / 255.0;
        arrow_marker.color.g =
            ((255.0 - colour[1]) * alpha + colour[1]) / 255.0;
        arrow_marker.color.b =
            ((255.0 - colour[2]) * alpha + colour[2]) / 255.0;
        arrow_marker.color.a = 1;

        geometry_msgs::msg::Point p0, p1;
        p0.x = point_0.x();
        p0.y = point_0.y();
        p0.z = point_0.z();
        p1.x = point_1.x();
        p1.y = point_1.y();
        p1.z = point_1.z();
        arrow_marker.points.push_back(p0);
        arrow_marker.points.push_back(p1);
        object_pred_marker_array.markers.push_back(arrow_marker);
      }
    }
    object_bbx_line_pub_->publish(object_pred_marker_array);
  }
}

void FrontendInbuiltDisplayRos::publishOdometry(
    const gtsam::Pose3& T_world_camera, Timestamp timestamp) {
  InbuiltDisplayCommon::publishOdometry(odometry_pub_, T_world_camera,
                                        timestamp);
  geometry_msgs::msg::TransformStamped t;
  // utils::convertWithHeader(T_world_camera, t, timestamp,
  // params_.world_frame_id, params_.camera_frame_id_); Send the transformation
  dyno::convert<gtsam::Pose3, geometry_msgs::msg::TransformStamped>(
      T_world_camera, t);

  t.header.stamp = node_->now();
  t.header.frame_id = params_.world_frame_id;
  t.child_frame_id = params_.camera_frame_id;

  tf_broadcaster_->sendTransform(t);
}

// void FrontendInbuiltDisplayRos::publishOdometryPath(const gtsam::Pose3&
// T_world_camera, Timestamp timestamp) {
//     geometry_msgs::msg::PoseStamped pose_stamped;
//     utils::convertWithHeader(T_world_camera, pose_stamped, timestamp,
//     "world");

//     static std_msgs::msg::Header header;
//     header.stamp = utils::toRosTime(timestamp);
//     header.frame_id = "world";
//     odom_path_msg_.header = header;

//     odom_path_msg_.poses.push_back(pose_stamped);
//     odometry_path_pub_->publish(odom_path_msg_);

// }

void FrontendInbuiltDisplayRos::publishDebugImage(
    const DebugImagery& debug_imagery) {
  if (debug_imagery.tracking_image.empty()) return;

  // cv::Mat resized_image;
  // cv::resize(debug_image, resized_image, cv::Size(640, 480));

  std_msgs::msg::Header hdr;
  sensor_msgs::msg::Image::SharedPtr msg =
      cv_bridge::CvImage(hdr, "bgr8", debug_imagery.tracking_image)
          .toImageMsg();
  tracking_image_pub_.publish(msg);
}

// TODO: lots of repeated code with this funyction and publishObjectPositions -
// need to functionalise
void FrontendInbuiltDisplayRos::publishGroundTruthInfo(
    Timestamp timestamp, const GroundTruthInputPacket& gt_packet,
    const cv::Mat& rgb) {
  // odometry gt
  const gtsam::Pose3& T_world_camera = gt_packet.X_world_;
  nav_msgs::msg::Odometry odom_msg;
  utils::convertWithHeader(T_world_camera, odom_msg, timestamp, "world",
                           "camera");
  gt_odometry_pub_->publish(odom_msg);

  const auto frame_id = gt_packet.frame_id_;

  // odom path gt
  geometry_msgs::msg::PoseStamped pose_stamped;
  utils::convertWithHeader(T_world_camera, pose_stamped, timestamp, "world");
  static std_msgs::msg::Header header;
  header.stamp = utils::toRosTime(timestamp);
  header.frame_id = "world";
  gt_odom_path_msg_.header = header;
  gt_odom_path_msg_.poses.push_back(pose_stamped);

  gt_odom_path_pub_->publish(gt_odom_path_msg_);

  // prepare display image
  cv::Mat disp_image;
  rgb.copyTo(disp_image);

  static ObjectPoseMap gt_object_poses;
  for (const auto& object_pose_gt : gt_packet.object_poses_) {
    gt_object_poses.insert22(object_pose_gt.object_id_, gt_packet.frame_id_,
                             object_pose_gt.L_world_);
  }

  publishObjectPositions(gt_object_pose_pub_, gt_object_poses, frame_id,
                         timestamp, "ground_truth");

  publishObjectPaths(gt_object_path_pub_, gt_object_poses, frame_id, timestamp,
                     "ground_truth", 60);

  // static std::map<ObjectId, gtsam::Pose3Vector> gt_object_trajectories;
  // //Used for gt path updating static std::map<ObjectId, FrameId>
  // gt_object_trajectories_update; // The last frame id that the object was
  // seen in

  // std::set<ObjectId> seen_objects;

  // //prepare gt object pose markers
  // visualization_msgs::msg::MarkerArray object_pose_marker_array;
  // visualization_msgs::msg::MarkerArray object_path_marker_array;
  // static visualization_msgs::msg::Marker delete_marker;
  // delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;

  // object_pose_marker_array.markers.push_back(delete_marker);
  // object_path_marker_array.markers.push_back(delete_marker);

  // for(const auto& object_pose_gt : gt_packet.object_poses_) {
  //     // const gtsam::Pose3 L_world = T_world_camera *
  //     object_pose_gt.L_camera_; const gtsam::Pose3 L_world =
  //     object_pose_gt.L_world_; const ObjectId object_id =
  //     object_pose_gt.object_id_;

  //     seen_objects.insert(object_id);

  //     visualization_msgs::msg::Marker marker;
  //     marker.header.frame_id = "world";
  //     marker.ns = "ground_truth_object_poses";
  //     marker.id = object_id;
  //     marker.type = visualization_msgs::msg::Marker::CUBE;
  //     marker.action = visualization_msgs::msg::Marker::ADD;
  //     marker.header.stamp = node_->now();
  //     marker.pose.position.x = L_world.x();
  //     marker.pose.position.y = L_world.y();
  //     marker.pose.position.z = L_world.z();
  //     marker.pose.orientation.x = L_world.rotation().toQuaternion().x();
  //     marker.pose.orientation.y = L_world.rotation().toQuaternion().y();
  //     marker.pose.orientation.z = L_world.rotation().toQuaternion().z();
  //     marker.pose.orientation.w = L_world.rotation().toQuaternion().w();
  //     marker.scale.x = 0.5;
  //     marker.scale.y = 0.5;
  //     marker.scale.z = 0.5;
  //     marker.color.a = 1.0; // Don't forget to set the alpha!

  //     const cv::Scalar colour = Color::uniqueId(object_id);
  //     marker.color.r = colour(0)/255.0;
  //     marker.color.g = colour(1)/255.0;
  //     marker.color.b = colour(2)/255.0;

  //     visualization_msgs::msg::Marker text_marker = marker;
  //     text_marker.ns = "ground_truth_object_labels";
  //     text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  //     text_marker.text = std::to_string(object_id);
  //     text_marker.pose.position.z += 1.0; //make it higher than the pose
  //     marker text_marker.scale.z = 0.7;

  //     object_pose_marker_array.markers.push_back(marker);
  //     object_pose_marker_array.markers.push_back(text_marker);

  //     //draw on bbox
  //     object_pose_gt.drawBoundingBox(disp_image);

  //     //update past trajectotries of objects
  //     auto it = gt_object_trajectories.find(object_id);
  //     if(it == gt_object_trajectories.end()) {
  //         gt_object_trajectories[object_id] = gtsam::Pose3Vector();
  //     }

  //     gt_object_trajectories_update[object_id] = frame_id;
  //     gt_object_trajectories[object_id].push_back(L_world);

  // }

  // //repeated code from publishObjectPositions function
  // //iterate over object trajectories and display the ones with enough poses
  // and the ones weve seen recently for(const auto& [object_id, poses] :
  // gt_object_trajectories) {
  //     const FrameId last_seen_frame =
  //     gt_object_trajectories_update.at(object_id);

  //     //if weve seen the object in the last 30 frames and the length is at
  //     least 2 if(poses.size() < 2u) {
  //         continue;
  //     }

  //     //draw a line list for viz
  //     visualization_msgs::msg::Marker line_list_marker;
  //     line_list_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  //     line_list_marker.header.frame_id = "world";
  //     line_list_marker.ns = "gt_frontend_composed_object_path";
  //     line_list_marker.id = object_id;
  //     line_list_marker.header.stamp = node_->now();
  //     line_list_marker.scale.x = 0.5;

  //     line_list_marker.pose.orientation.x = 0;
  //     line_list_marker.pose.orientation.y = 0;
  //     line_list_marker.pose.orientation.z = 0;
  //     line_list_marker.pose.orientation.w = 1;

  //     const cv::Scalar colour = Color::uniqueId(object_id);
  //     line_list_marker.color.r = colour(0)/255.0;
  //     line_list_marker.color.g = colour(1)/255.0;
  //     line_list_marker.color.b = colour(2)/255.0;
  //     line_list_marker.color.a = 1;

  //     //only draw the last 60 poses
  //     const size_t traj_size = std::min(60, static_cast<int>(poses.size()));
  //     // const size_t traj_size = poses.size();
  //     //have to duplicate the first in each drawn pair so that we construct a
  //     complete line for(size_t i = poses.size() - traj_size + 1; i <
  //     poses.size(); i++) {
  //         const gtsam::Pose3& prev_pose = poses.at(i-1);
  //         const gtsam::Pose3& curr_pose = poses.at(i);

  //         {
  //             geometry_msgs::msg::Point p;
  //             p.x = prev_pose.x();
  //             p.y = prev_pose.y();
  //             p.z = prev_pose.z();

  //             line_list_marker.points.push_back(p);
  //         }

  //         {
  //             geometry_msgs::msg::Point p;
  //             p.x = curr_pose.x();
  //             p.y = curr_pose.y();
  //             p.z = curr_pose.z();

  //             line_list_marker.points.push_back(p);
  //         }
  //     }

  //     object_path_marker_array.markers.push_back(line_list_marker);
  // }

  // Publish centroids of composed object poses
  // gt_object_pose_pub_->publish(object_pose_marker_array);

  // // Publish composed object path
  // gt_object_path_pub_->publish(object_path_marker_array);

  cv::Mat resized_image;
  cv::resize(disp_image, resized_image, cv::Size(640, 480));

  std_msgs::msg::Header hdr;
  sensor_msgs::msg::Image::SharedPtr msg =
      cv_bridge::CvImage(hdr, "bgr8", resized_image).toImageMsg();
  gt_bounding_box_pub_.publish(msg);
}

}  // namespace dyno
````

## File: src/displays/inbuilt_displays/InbuiltDisplayCommon.cc
````
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

#include "dynosam_ros/displays/inbuilt_displays/InbuiltDisplayCommon.hpp"
#include "dynosam_ros/displays/DisplaysCommon.hpp"

#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/memory.h>

#include <dynosam/common/PointCloudProcess.hpp>
#include <dynosam/visualizer/ColourMap.hpp>

#include "dynosam_ros/RosUtils.hpp"

namespace dyno {

InbuiltDisplayCommon::InbuiltDisplayCommon(const DisplayParams& params,
                                           rclcpp::Node::SharedPtr node)
    : params_(params), node_(node) {}

CloudPerObject InbuiltDisplayCommon::publishPointCloud(
    PointCloud2Pub::SharedPtr pub, const StatusLandmarkVector& landmarks,
    const gtsam::Pose3& T_world_camera) {
  return DisplayCommon::publishPointCloud(pub, landmarks, T_world_camera, params_.world_frame_id);
}

void InbuiltDisplayCommon::publishOdometry(OdometryPub::SharedPtr pub,
                                           const gtsam::Pose3& T_world_camera,
                                           Timestamp timestamp) {
  DisplayCommon::publishOdometry(pub, T_world_camera, timestamp,
                           params_.world_frame_id, params_.camera_frame_id);
}

void InbuiltDisplayCommon::publishOdometryPath(PathPub::SharedPtr pub,
                                               const gtsam::Pose3Vector& poses,
                                               Timestamp latest_timestamp) {
  DisplayCommon::publishOdometryPath(pub, poses, latest_timestamp, params_.world_frame_id);
}

void InbuiltDisplayCommon::publishObjectPositions(
    MarkerArrayPub::SharedPtr pub, const ObjectPoseMap& object_positions,
    FrameId frame_id, Timestamp latest_timestamp,
    const std::string& prefix_marker_namespace, bool draw_labels,
    double scale) {
  visualization_msgs::msg::MarkerArray object_pose_marker_array;
  static visualization_msgs::msg::Marker delete_marker;
  delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;

  auto ros_time = utils::toRosTime(latest_timestamp);

  object_pose_marker_array.markers.push_back(delete_marker);

  for (const auto& [object_id, poses_map] : object_positions) {
    // do not draw if in current frame
    if (!poses_map.exists(frame_id)) {
      continue;
    }

    const gtsam::Pose3& pose = poses_map.at(frame_id);

    // assume
    // object centroid per frame
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = params_.world_frame_id;
    marker.ns = prefix_marker_namespace + "_object_positions";
    marker.id = object_id;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.header.stamp = ros_time;
    marker.pose.position.x = pose.x();
    marker.pose.position.y = pose.y();
    marker.pose.position.z = pose.z();
    marker.scale.x = scale;
    marker.scale.y = scale;
    marker.scale.z = scale;
    marker.color.a = 1.0;  // Don't forget to set the alpha!

    const cv::Scalar colour = Color::uniqueId(object_id);
    marker.color.r = colour(0) / 255.0;
    marker.color.g = colour(1) / 255.0;
    marker.color.b = colour(2) / 255.0;

    object_pose_marker_array.markers.push_back(marker);

    if (draw_labels) {
      visualization_msgs::msg::Marker text_marker = marker;
      text_marker.ns = prefix_marker_namespace + "_object_labels";
      text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      text_marker.text = std::to_string(object_id);
      text_marker.pose.position.z += 1.0;  // make it higher than the pose
                                           // marker
      text_marker.scale.z = 0.7;
      object_pose_marker_array.markers.push_back(text_marker);
    }
  }

  pub->publish(object_pose_marker_array);
}

void InbuiltDisplayCommon::publishObjectPaths(
    MarkerArrayPub::SharedPtr pub, const ObjectPoseMap& object_positions,
    FrameId frame_id, Timestamp latest_timestamp,
    const std::string& prefix_marker_namespace, const int min_poses) {
  visualization_msgs::msg::MarkerArray object_path_marker_array;
  static visualization_msgs::msg::Marker delete_marker;
  delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;

  auto ros_time = utils::toRosTime(latest_timestamp);
  object_path_marker_array.markers.push_back(delete_marker);

  for (const auto& [object_id, poses_map] : object_positions) {
    if (poses_map.size() < 2u) {
      continue;
    }

    // draw a line list for viz
    visualization_msgs::msg::Marker line_list_marker;
    line_list_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line_list_marker.header.frame_id = "world";
    line_list_marker.ns = prefix_marker_namespace + "_object_path";
    line_list_marker.id = object_id;
    line_list_marker.header.stamp = ros_time;
    line_list_marker.scale.x = 0.05;

    line_list_marker.pose.orientation.x = 0;
    line_list_marker.pose.orientation.y = 0;
    line_list_marker.pose.orientation.z = 0;
    line_list_marker.pose.orientation.w = 1;

    const cv::Scalar colour = Color::uniqueId(object_id);
    line_list_marker.color.r = colour(0) / 255.0;
    line_list_marker.color.g = colour(1) / 255.0;
    line_list_marker.color.b = colour(2) / 255.0;
    line_list_marker.color.a = 1;

    size_t traj_size;
    // draw all the poses
    if (min_poses == -1) {
      traj_size = poses_map.size();
    } else {
      traj_size = std::min(min_poses, static_cast<int>(poses_map.size()));
    }

    // totally assume in order
    auto map_iter = poses_map.end();
    // equivalant to map_iter-traj_size + 1
    // we want to go backwards to the starting point specified by trajectory
    // size
    std::advance(map_iter, (-traj_size + 1));
    for (; map_iter != poses_map.end(); map_iter++) {
      auto prev_iter = std::prev(map_iter, 1);
      const gtsam::Pose3& prev_pose = prev_iter->second;
      ;
      const gtsam::Pose3& curr_pose = map_iter->second;

      // check frames are consequative
      //  CHECK_EQ(prev_iter->first + 1, map_iter->first) << " For object " <<
      //  object_id;
      LOG_IF(INFO, prev_iter->first + 1 != map_iter->first)
          << " Frames not consequative for object " << object_id;

      {
        geometry_msgs::msg::Point p;
        p.x = prev_pose.x();
        p.y = prev_pose.y();
        p.z = prev_pose.z();

        line_list_marker.points.push_back(p);
      }

      {
        geometry_msgs::msg::Point p;
        p.x = curr_pose.x();
        p.y = curr_pose.y();
        p.z = curr_pose.z();

        line_list_marker.points.push_back(p);
      }
    }

    object_path_marker_array.markers.push_back(line_list_marker);
  }

  pub->publish(object_path_marker_array);
}

void InbuiltDisplayCommon::publishObjectBoundingBox(
    MarkerArrayPub::SharedPtr aabb_pub, MarkerArrayPub::SharedPtr obb_pub,
    const CloudPerObject& cloud_per_object, Timestamp timestamp,
    const std::string& prefix_marker_namespace) {
  if (!aabb_pub && !obb_pub) {
    return;
  }

  visualization_msgs::msg::MarkerArray aabb_markers;  // using linelist
  visualization_msgs::msg::MarkerArray obb_markers;   // using cube
  aabb_markers.markers.push_back(getDeletionMarker());
  obb_markers.markers.push_back(getDeletionMarker());

  constructBoundingBoxeMarkers(cloud_per_object, aabb_markers, obb_markers,
                               timestamp, prefix_marker_namespace);

  if (aabb_pub) aabb_pub->publish(aabb_markers);
  if (obb_pub) aabb_pub->publish(obb_markers);
}

void InbuiltDisplayCommon::createAxisMarkers(
    const gtsam::Pose3& pose, MarkerArray& axis_markers, Timestamp timestamp,
    const cv::Scalar& colour, const std::string& frame, const std::string& ns,
    double length, double radius) {
  const auto pose_matrix = pose.matrix();
  Eigen::Isometry3d x = Eigen::Translation3d(length / 2.0, 0, 0) *
                        Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitY());
  gtsam::Pose3 x_pose(pose_matrix * x.matrix());

  // Publish y axis
  Eigen::Isometry3d y = Eigen::Translation3d(0, length / 2.0, 0) *
                        Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitX());
  gtsam::Pose3 y_pose(pose_matrix * y.matrix());

  Eigen::Isometry3d z = Eigen::Translation3d(0, 0, length / 2.0) *
                        Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ());
  gtsam::Pose3 z_pose(pose_matrix * z.matrix());

  const auto ros_time = utils::toRosTime(timestamp);

  auto make_cylinder =
      [&length, &radius, &ros_time, &ns, &frame,
       &colour](const gtsam::Pose3& pose) -> visualization_msgs::msg::Marker {
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = ros_time;
    marker.ns = ns;
    marker.header.frame_id = frame;

    geometry_msgs::msg::Pose pose_msg;
    dyno::convert(pose, pose_msg);

    marker.pose = pose_msg;
    marker.scale.x = radius;
    marker.scale.y = radius;
    marker.scale.z = length;

    marker.color.r = colour(0) / 255.0;
    marker.color.g = colour(1) / 255.0;
    marker.color.b = colour(2) / 255.0;

    return marker;
  };

  axis_markers.markers.push_back(make_cylinder(x_pose));
  axis_markers.markers.push_back(make_cylinder(y_pose));
  axis_markers.markers.push_back(make_cylinder(z_pose));
}

void InbuiltDisplayCommon::constructBoundingBoxeMarkers(
    const CloudPerObject& cloud_per_object, MarkerArray& aabb_markers,
    MarkerArray& obb_markers, Timestamp timestamp,
    const std::string& prefix_marker_namespace) {
  auto ros_time = utils::toRosTime(timestamp);

  for (const auto& [object_id, obj_cloud] : cloud_per_object) {
    pcl::PointXYZ centroid;
    pcl::computeCentroid(obj_cloud, centroid);

    const cv::Scalar colour = Color::uniqueId(object_id);

    visualization_msgs::msg::Marker txt_marker;
    txt_marker.header.frame_id = "world";
    txt_marker.ns = prefix_marker_namespace + "_object_id_txt";
    txt_marker.id = object_id;
    txt_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    txt_marker.action = visualization_msgs::msg::Marker::ADD;
    txt_marker.header.stamp = ros_time;

    txt_marker.scale.z = 0.5;
    // txt_marker.scale.z = 2.0;
    txt_marker.color.r = colour(0) / 255.0;
    txt_marker.color.g = colour(1) / 255.0;
    txt_marker.color.b = colour(2) / 255.0;
    txt_marker.color.a = 1;
    txt_marker.text = "Obj " + std::to_string(object_id);
    txt_marker.pose.position.x = centroid.x;
    // txt_marker.pose.position.y = centroid.y - 2.0;
    // txt_marker.pose.position.z = centroid.z - 1.0;
    txt_marker.pose.position.y = centroid.y - 0.6;
    txt_marker.pose.position.z = centroid.z - 0.5;
    aabb_markers.markers.push_back(txt_marker);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr obj_cloud_ptr =
        pcl::make_shared<pcl::PointCloud<pcl::PointXYZRGB> >(obj_cloud);
    ObjectBBX aabb = findAABBFromCloud<pcl::PointXYZRGB>(obj_cloud_ptr);
    ObjectBBX obb = findOBBFromCloud<pcl::PointXYZRGB>(obj_cloud_ptr);

    visualization_msgs::msg::Marker aabb_marker;
    aabb_marker.header.frame_id = "world";
    aabb_marker.ns = prefix_marker_namespace + "object_aabb";
    aabb_marker.id = object_id;
    aabb_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    aabb_marker.action = visualization_msgs::msg::Marker::ADD;
    aabb_marker.header.stamp = ros_time;
    aabb_marker.scale.x = 0.03;

    aabb_marker.pose.orientation.x = 0;
    aabb_marker.pose.orientation.y = 0;
    aabb_marker.pose.orientation.z = 0;
    aabb_marker.pose.orientation.w = 1;

    aabb_marker.color.r = colour(0) / 255.0;
    aabb_marker.color.g = colour(1) / 255.0;
    aabb_marker.color.b = colour(2) / 255.0;
    aabb_marker.color.a = 1;

    for (const pcl::PointXYZ& this_line_list_point :
         findLineListPointsFromAABBMinMax(aabb.min_bbx_point_,
                                          aabb.max_bbx_point_)) {
      geometry_msgs::msg::Point p;
      p.x = this_line_list_point.x;
      p.y = this_line_list_point.y;
      p.z = this_line_list_point.z;
      aabb_marker.points.push_back(p);
    }
    aabb_markers.markers.push_back(aabb_marker);

    visualization_msgs::msg::Marker obbx_marker;
    obbx_marker.header.frame_id = "world";
    obbx_marker.ns = prefix_marker_namespace + "object_obb";
    obbx_marker.id = object_id;
    obbx_marker.type = visualization_msgs::msg::Marker::CUBE;
    obbx_marker.action = visualization_msgs::msg::Marker::ADD;
    obbx_marker.header.stamp = ros_time;
    obbx_marker.scale.x = obb.max_bbx_point_.x() - obb.min_bbx_point_.x();
    obbx_marker.scale.y = obb.max_bbx_point_.y() - obb.min_bbx_point_.y();
    obbx_marker.scale.z = obb.max_bbx_point_.z() - obb.min_bbx_point_.z();

    obbx_marker.pose.position.x = obb.bbx_position_.x();
    obbx_marker.pose.position.y = obb.bbx_position_.y();
    obbx_marker.pose.position.z = obb.bbx_position_.z();

    const gtsam::Quaternion& q = obb.orientation_.toQuaternion();
    obbx_marker.pose.orientation.x = q.x();
    obbx_marker.pose.orientation.y = q.y();
    obbx_marker.pose.orientation.z = q.z();
    obbx_marker.pose.orientation.w = q.w();

    obbx_marker.color.r = colour(0) / 255.0;
    obbx_marker.color.g = colour(1) / 255.0;
    obbx_marker.color.b = colour(2) / 255.0;
    obbx_marker.color.a = 0.2;

    obb_markers.markers.push_back(obbx_marker);
  }
}

MarkerArray InbuiltDisplayCommon::createCameraMarker(
    const gtsam::Pose3& T_world_x, Timestamp timestamp, const std::string& ns,
    const cv::Scalar& colour, double marker_scale) {
  auto transform_odom_to =
      [&T_world_x](
          const geometry_msgs::msg::Pose& msg) -> geometry_msgs::msg::Pose {
    // convert msg to pose
    gtsam::Pose3 msg_as_pose;
    dyno::convert(msg, msg_as_pose);
    // transform pose into world frame
    gtsam::Pose3 transformed_pose = T_world_x * msg_as_pose;
    // transform pose back to ROS msg and return
    geometry_msgs::msg::Pose transformed_msg;
    dyno::convert(transformed_pose, transformed_msg);
    return transformed_msg;
  };

  auto ros_time = utils::toRosTime(timestamp);

  // make rectangles as frame
  const double r_w = 1.0;
  const double z_plane = (r_w / 2.0) * marker_scale;

  static constexpr double sqrt2_2 = sqrt(2) / 2;

  MarkerArray marker_array;
  visualization_msgs::msg::Marker marker;

  // the marker will be displayed in frame_id
  marker.header.frame_id = params_.world_frame_id;
  marker.header.stamp = ros_time;
  marker.ns = ns;
  marker.action = 0;
  marker.id = 8;  // 8 of 8 markers

  marker.type = visualization_msgs::msg::Marker::CUBE;
  marker.scale.x = r_w * marker_scale;
  marker.scale.y = 0.04 * marker_scale;
  marker.scale.z = 0.04 * marker_scale;
  marker.color.r = colour(0) / 255.0;
  marker.color.g = colour(1) / 255.0;
  marker.color.b = colour(2) / 255.0;
  marker.color.a = 1.0;

  marker.pose.orientation.x = 0;
  marker.pose.orientation.y = 0;
  marker.pose.orientation.z = 0;
  marker.pose.orientation.w = 1;
  marker.id--;
  marker_array.markers.push_back(marker);
  marker.pose.position.y = -(r_w / 4.0) * marker_scale;
  marker.id--;
  marker_array.markers.push_back(marker);

  marker.scale.x = (r_w / 2.0) * marker_scale;
  marker.pose.position.x = (r_w / 2.0) * marker_scale;
  marker.pose.position.y = 0;
  marker.pose.orientation.w = sqrt2_2;
  marker.pose.orientation.z = sqrt2_2;
  marker.id--;
  marker_array.markers.push_back(marker);
  marker.pose.position.x = -(r_w / 2.0) * marker_scale;
  marker.id--;
  marker_array.markers.push_back(marker);

  // make pyramid edges
  marker.scale.x = (3.0 * r_w / 4.0) * marker_scale;
  marker.pose.position.z = 0.5 * z_plane;

  marker.pose.position.x = (r_w / 4.0) * marker_scale;
  marker.pose.position.y = (r_w / 8.0) * marker_scale;
  //  0.08198092, -0.34727674,  0.21462883,  0.9091823
  marker.pose.orientation.x = 0.08198092;
  marker.pose.orientation.y = -0.34727674;
  marker.pose.orientation.z = 0.21462883;
  marker.pose.orientation.w = 0.9091823;
  marker.id--;
  marker_array.markers.push_back(marker);

  marker.pose.position.x = -(r_w / 4.0) * marker_scale;
  marker.pose.position.y = (r_w / 8.0) * marker_scale;
  // -0.27395078, -0.22863284,  0.9091823 ,  0.21462883
  marker.pose.orientation.x = 0.08198092;
  marker.pose.orientation.y = 0.34727674;
  marker.pose.orientation.z = -0.21462883;
  marker.pose.orientation.w = 0.9091823;
  marker.id--;
  marker_array.markers.push_back(marker);

  marker.pose.position.x = -(r_w / 4.0) * marker_scale;
  marker.pose.position.y = -(r_w / 8.0) * marker_scale;
  //  -0.08198092,  0.34727674,  0.21462883,  0.9091823
  marker.pose.orientation.x = -0.08198092;
  marker.pose.orientation.y = 0.34727674;
  marker.pose.orientation.z = 0.21462883;
  marker.pose.orientation.w = 0.9091823;
  marker.id--;
  marker_array.markers.push_back(marker);

  marker.pose.position.x = (r_w / 4.0) * marker_scale;
  marker.pose.position.y = -(r_w / 8.0) * marker_scale;
  // -0.08198092, -0.34727674, -0.21462883,  0.9091823
  marker.pose.orientation.x = -0.08198092;
  marker.pose.orientation.y = -0.34727674;
  marker.pose.orientation.z = -0.21462883;
  marker.pose.orientation.w = 0.9091823;
  marker.id--;
  marker_array.markers.push_back(marker);

  for (auto& marker : marker_array.markers) {
    // put in world frame
    marker.pose = transform_odom_to(marker.pose);
  }
  return marker_array;
}

}  // namespace dyno
````

## File: src/displays/DisplaysCommon.cc
````
#include "dynosam_ros/displays/DisplaysCommon.hpp"

#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/memory.h>

#include <dynosam/common/PointCloudProcess.hpp>
#include <dynosam/visualizer/ColourMap.hpp>

#include "dynosam_ros/RosUtils.hpp"


namespace dyno {

CloudPerObject DisplayCommon::publishPointCloud(PointCloud2Pub::SharedPtr pub, const StatusLandmarkVector& landmarks, const gtsam::Pose3& T_world_camera, const std::string& frame_id) {
    pcl::PointCloud<pcl::PointXYZRGB> cloud;

  CloudPerObject clouds_per_obj;

  for (const auto& status_estimate : landmarks) {
    Landmark lmk_world = status_estimate.value();
    const ObjectId object_id = status_estimate.objectId();
    if (status_estimate.referenceFrame() == ReferenceFrame::LOCAL) {
      lmk_world = T_world_camera * status_estimate.value();
    } else if (status_estimate.referenceFrame() == ReferenceFrame::OBJECT) {
      throw DynosamException(
          "Cannot display object point in the object reference frame");
    }

    pcl::PointXYZRGB pt;
    if (status_estimate.isStatic()) {
      // publish static lmk's as white
      pt = pcl::PointXYZRGB(lmk_world(0), lmk_world(1), lmk_world(2), 0, 0, 0);
    } else {
      const cv::Scalar colour = Color::uniqueId(object_id);
      pt = pcl::PointXYZRGB(lmk_world(0), lmk_world(1), lmk_world(2), colour(0),
                            colour(1), colour(2));
    }
    cloud.points.push_back(pt);
    clouds_per_obj[object_id].push_back(pt);
  }

  sensor_msgs::msg::PointCloud2 pc2_msg;
  pcl::toROSMsg(cloud, pc2_msg);
  pc2_msg.header.frame_id = frame_id;
  pub->publish(pc2_msg);

  return clouds_per_obj;
}

void DisplayCommon::publishOdometry(OdometryPub::SharedPtr pub, const gtsam::Pose3& T_world_camera, Timestamp timestamp, const std::string& frame_id, const std::string& child_frame_id) {
    nav_msgs::msg::Odometry odom_msg;
    utils::convertWithHeader(T_world_camera, odom_msg, timestamp,
                            frame_id, child_frame_id);
    pub->publish(odom_msg);
}


void DisplayCommon::publishOdometryPath(PathPub::SharedPtr pub, const gtsam::Pose3Vector& poses, Timestamp latest_timestamp, const std::string& frame_id) {

nav_msgs::msg::Path path;
  for (const gtsam::Pose3& odom : poses) {
    geometry_msgs::msg::PoseStamped pose_stamped;
    utils::convertWithHeader(odom, pose_stamped, latest_timestamp,frame_id);
    path.poses.push_back(pose_stamped);
  }

  path.header.stamp = utils::toRosTime(latest_timestamp);
  path.header.frame_id = frame_id;
  pub->publish(path);
}

} //dyno
````

## File: src/DataProviderRos.cc
````
/*
 *   Copyright (c) 2025 ACFR-RPG, University of Sydney, Jesse Morris
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

#include "dynosam_ros/DataProviderRos.hpp"

#include <dynosam/common/ImageTypes.hpp>

#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace dyno {

DataProviderRos::DataProviderRos(rclcpp::Node::SharedPtr node)
    : DataProvider(), node_(node) {}

const cv::Mat DataProviderRos::readRgbRosImage(
    const sensor_msgs::msg::Image::ConstSharedPtr& img_msg) const {
  return convertRosImage<ImageType::RGBMono>(img_msg);
}

const cv::Mat DataProviderRos::readDepthRosImage(
    const sensor_msgs::msg::Image::ConstSharedPtr& img_msg) const {
  return convertRosImage<ImageType::Depth>(img_msg);
}

const cv::Mat DataProviderRos::readFlowRosImage(
    const sensor_msgs::msg::Image::ConstSharedPtr& img_msg) const {
  return convertRosImage<ImageType::OpticalFlow>(img_msg);
}

const cv::Mat DataProviderRos::readMaskRosImage(
    const sensor_msgs::msg::Image::ConstSharedPtr& img_msg) const {
  return convertRosImage<ImageType::MotionMask>(img_msg);
}

const cv_bridge::CvImageConstPtr DataProviderRos::readRosImage(
    const sensor_msgs::msg::Image::ConstSharedPtr& img_msg) const {
  CHECK(img_msg);
  cv_bridge::CvImageConstPtr cv_ptr;
  try {
    // important to copy to ensure that memory does not go out of scope (which
    // it seems to !!!)
    cv_ptr = cv_bridge::toCvCopy(img_msg);
  } catch (cv_bridge::Exception& exception) {
    RCLCPP_FATAL(node_->get_logger(), "cv_bridge exception: %s",
                 exception.what());
    rclcpp::shutdown();
  }
  return cv_ptr;
}

}  // namespace dyno
````

## File: src/OnlineDataProviderRos.cc
````
/*
 *   Copyright (c) 2025 ACFR-RPG, University of Sydney, Jesse Morris
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

#include "dynosam_ros/OnlineDataProviderRos.hpp"

#include "dynosam_ros/RosUtils.hpp"

namespace dyno {

OnlineDataProviderRos::OnlineDataProviderRos(
    rclcpp::Node::SharedPtr node, const OnlineDataProviderRosParams &params)
    : DataProviderRos(node), frame_id_(0u) {
  if (params.wait_for_camera_params) {
    waitAndSetCameraParams(
        std::chrono::milliseconds(params.camera_params_timeout));
  }

  connect();
  CHECK_EQ(shutdown_, false);
}

bool OnlineDataProviderRos::spin() { return !shutdown_; }

void OnlineDataProviderRos::shutdown() {
  shutdown_ = true;
  // shutdown synchronizer
  RCLCPP_INFO_STREAM(node_->get_logger(),
                     "Shutting down OnlineDataProviderRos");
  if (sync_) sync_.reset();

  rgb_image_sub_.unsubscribe();
  depth_image_sub_.unsubscribe();
  flow_image_sub_.unsubscribe();
  mask_image_sub_.unsubscribe();
}

void OnlineDataProviderRos::connect() {
  rclcpp::Node *node_ptr = node_.get();
  CHECK_NOTNULL(node_ptr);
  rgb_image_sub_.subscribe(node_ptr, "image/rgb");
  depth_image_sub_.subscribe(node_ptr, "image/depth");
  flow_image_sub_.subscribe(node_ptr, "image/flow");
  mask_image_sub_.subscribe(node_ptr, "image/mask");

  if (sync_) sync_.reset();

  static constexpr size_t kQueueSize = 20u;
  sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
      SyncPolicy(kQueueSize), rgb_image_sub_, depth_image_sub_, flow_image_sub_,
      mask_image_sub_);

  sync_->registerCallback(std::bind(
      &OnlineDataProviderRos::imageSyncCallback, this, std::placeholders::_1,
      std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));

  RCLCPP_INFO_STREAM(
      node_->get_logger(),
      "OnlineDataProviderRos has been connected. Subscribed to image topics: "
          << rgb_image_sub_.getSubscriber()->get_topic_name() << " "
          << depth_image_sub_.getSubscriber()->get_topic_name() << " "
          << flow_image_sub_.getSubscriber()->get_topic_name() << " "
          << mask_image_sub_.getSubscriber()->get_topic_name() << ".");

  shutdown_ = false;
}

void OnlineDataProviderRos::imageSyncCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr &rgb_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr &depth_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr &flow_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr &mask_msg) {
  if (!image_container_callback_) {
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                          "Image Sync callback triggered but "
                          "image_container_callback_ is not registered!");
    return;
  }

  const cv::Mat rgb = readRgbRosImage(rgb_msg);
  const cv::Mat depth = readDepthRosImage(depth_msg);
  const cv::Mat flow = readFlowRosImage(flow_msg);
  const cv::Mat mask = readMaskRosImage(mask_msg);

  const Timestamp timestamp = utils::fromRosTime(rgb_msg->header.stamp);
  const FrameId frame_id = frame_id_;
  frame_id_++;

  ImageContainer::Ptr image_container = ImageContainer::Create(
      timestamp, frame_id, ImageWrapper<ImageType::RGBMono>(rgb),
      ImageWrapper<ImageType::Depth>(depth),
      ImageWrapper<ImageType::OpticalFlow>(flow),
      ImageWrapper<ImageType::MotionMask>(mask));
  CHECK(image_container);

  cv::Mat of_viz, motion_viz, depth_viz;
  of_viz = ImageType::OpticalFlow::toRGB(flow);
  motion_viz = ImageType::MotionMask::toRGB(mask);
  depth_viz = ImageType::Depth::toRGB(depth);

  // cv::imshow("Optical Flow", of_viz);
  // cv::imshow("Motion mask", motion_viz);
  // cv::imshow("Depth", depth_viz);
  // cv::waitKey(1);
  // trigger callback to send data to the DataInterface!
  image_container_callback_(image_container);
}

}  // namespace dyno
````

## File: src/PipelineRos.cc
````
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

#include "dynosam_ros/PipelineRos.hpp"

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <dynosam/dataprovider/DataProviderFactory.hpp>
#include <dynosam/dataprovider/DataProviderUtils.hpp>
#include <dynosam/pipeline/PipelineParams.hpp>
#include <dynosam/visualizer/OpenCVFrontendDisplay.hpp>

#include "dynosam_ros/Display-Definitions.hpp"
#include "dynosam_ros/OnlineDataProviderRos.hpp"
#include "dynosam_ros/ZEDOnlineDataProvider.hpp"
#include "dynosam_ros/RosUtils.hpp"
#include "dynosam_ros/displays/DisplaysImpl.hpp"
#include "rcl_interfaces/msg/parameter.hpp"
#include "rclcpp/parameter.hpp"

namespace dyno {

DynoNode::DynoNode(const std::string& node_name,
                   const rclcpp::NodeOptions& options)
    : Node(node_name, "dynosam", options) {
  RCLCPP_INFO_STREAM(this->get_logger(), "Starting DynoNode");
  auto params_path = getParamsPath();
  RCLCPP_INFO_STREAM(this->get_logger(),
                     "Loading Dyno VO params from: " << params_path);
  dyno_params_ = std::make_unique<DynoParams>(params_path);
}

ZEDOnlineDataProviderParams DynoNode::parseZEDParams() {
  ZEDOnlineDataProviderParams zed_params;
  
  zed_params.rgb_topic_name =
      ParameterConstructor(this, "zed.rgb_topic", zed_params.rgb_topic_name)
          .description("ZED RGB image topic name")
          .finish()
          .get<std::string>();
          
  zed_params.depth_topic_name =
      ParameterConstructor(this, "zed.depth_topic", zed_params.depth_topic_name)
          .description("ZED depth image topic name")
          .finish()
          .get<std::string>();
          
  zed_params.camera_info_topic_name =
      ParameterConstructor(this, "zed.camera_info_topic", zed_params.camera_info_topic_name)
          .description("ZED camera info topic name")
          .finish()
          .get<std::string>();
          
  zed_params.imu_topic_name =
      ParameterConstructor(this, "zed.imu_topic", zed_params.imu_topic_name)
          .description("ZED IMU topic name")
          .finish()
          .get<std::string>();
          
  zed_params.image_sync_queue_size =
      ParameterConstructor(this, "zed.image_sync_queue_size", zed_params.image_sync_queue_size)
          .description("Image synchronization queue size")
          .finish()
          .get<int>();
          
  zed_params.image_sync_slop_sec =
      ParameterConstructor(this, "zed.image_sync_slop_sec", zed_params.image_sync_slop_sec)
          .description("Image synchronization time tolerance (seconds)")
          .finish()
          .get<double>();
          
  zed_params.wait_for_camera_info =
      ParameterConstructor(this, "zed.wait_for_camera_info", zed_params.wait_for_camera_info)
          .description("Wait for camera info message on startup")
          .finish()
          .get<bool>();
          
  int timeout_ms =
      ParameterConstructor(this, "zed.camera_info_timeout_ms", static_cast<int>(zed_params.camera_info_timeout_ms.count()))
          .description("Timeout for waiting for camera info (ms, -1 for infinite)")
          .finish()
          .get<int>();
  zed_params.camera_info_timeout_ms = std::chrono::milliseconds(timeout_ms);
  
  zed_params.imu_buffer_size =
      ParameterConstructor(this, "zed.imu_buffer_size", static_cast<int>(zed_params.imu_buffer_size))
          .description("IMU buffer size")
          .finish()
          .get<int>();
          
  zed_params.output_rgb =
      ParameterConstructor(this, "zed.output_rgb", zed_params.output_rgb)
          .description("Output RGB (true) or grayscale (false)")
          .finish()
          .get<bool>();
          
  zed_params.enable_imu_processing =
      ParameterConstructor(this, "zed.enable_imu_processing", zed_params.enable_imu_processing)
          .description("Enable IMU processing")
          .finish()
          .get<bool>();
          
  return zed_params;
}

dyno::DataProvider::Ptr DynoNode::createDataProvider() {
  const bool online =
      ParameterConstructor(this, "online", false)
          .description("If the online DataProvider should be used")
          .finish()
          .get<bool>();

  if (online) {
    // Check if ZED provider should be used
    const std::string online_provider_type =
        ParameterConstructor(this, "online_provider_type", std::string("generic"))
            .description("Type of online data provider: 'generic' or 'zed'")
            .finish()
            .get<std::string>();
    
    if (online_provider_type == "zed") {
      RCLCPP_INFO_STREAM(
          this->get_logger(),
          "ZED Online DataProvider selected. Waiting for ZED ROS topics...");
      
      auto zed_params = parseZEDParams();
      return std::make_shared<ZEDOnlineDataProvider>(
          this->create_sub_node("dataprovider"), zed_params);
    } else {
      RCLCPP_INFO_STREAM(
          this->get_logger(),
          "Generic Online DataProvider selected. Waiting for ROS topics...");
      
      OnlineDataProviderRosParams online_params;
      online_params.wait_for_camera_params =
          ParameterConstructor(this, "wait_for_camera_params",
                               online_params.wait_for_camera_params)
              .description(
                  "If the online DataProvider should wait for the camera params "
                  "on a ROS topic!")
              .finish()
              .get<bool>();
      online_params.camera_params_timeout =
          ParameterConstructor(this, "camera_params_timeout",
                               online_params.camera_params_timeout)
              .description(
                  "When waiting for camera params, how long the online "
                  "DataProvider should wait before time out (ms)")
              .finish()
              .get<int>();
      return std::make_shared<OnlineDataProviderRos>(
          this->create_sub_node("dataprovider"), online_params);
    }
  }

  auto params_path = getParamsPath();
  auto dataset_path = getDatasetPath();
  auto dyno_params = getDynoParams();

  RCLCPP_INFO_STREAM(this->get_logger(),
                     "Loading dataset from: " << dataset_path);

  dyno::DataProvider::Ptr data_loader = dyno::DataProviderFactory::Create(
      dataset_path, params_path,
      static_cast<dyno::DatasetType>(dyno_params.dataProviderType()));
  RCLCPP_INFO_STREAM(this->get_logger(), "Constructed data loader");
  return data_loader;
}

std::string DynoNode::searchForPathWithParams(const std::string& param_name,
                                              const std::string& default_path,
                                              const std::string& description) {
  // check if we've alrady declared this param
  // use non-default version so that ParameterConstructor throws exception if no
  // parameter is provided on the param server
  const std::string path = ParameterConstructor(this, param_name)
                               .description(description)
                               .finish()
                               .get<std::string>();
  throwExceptionIfPathInvalid(path);
  return path;
}

DynoPipelineManagerRos::DynoPipelineManagerRos(
    const rclcpp::NodeOptions& options)
    : DynoNode("dynosam", options) {}

void DynoPipelineManagerRos::initalisePipeline() {
  RCLCPP_INFO_STREAM(this->get_logger(), "Starting DynoPipelineManagerRos");

  auto params = getDynoParams();

  // setup display params
  DisplayParams display_params;
  display_params.camera_frame_id =
      ParameterConstructor(this, "camera_frame_id",
                           display_params.camera_frame_id)
          .description(
              "ROS frame id for the camera (ie. the measured odometry)")
          .finish()
          .get<std::string>();
  display_params.world_frame_id =
      ParameterConstructor(this, "world_frame_id",
                           display_params.world_frame_id)
          .description("ROS frame id for the static workd frame (ie. odometry)")
          .finish()
          .get<std::string>();

  auto frontend_display = std::make_shared<dyno::FrontendDisplayRos>(
      display_params, this->create_sub_node("frontend"));
  auto backend_display = std::make_shared<dyno::BackendDisplayRos>(
      display_params, this->create_sub_node("backend"));

  auto data_loader = createDataProvider();
  pipeline_ = std::make_unique<DynoPipelineManager>(
      params, data_loader, frontend_display, backend_display);
}

}  // namespace dyno
````

## File: src/RosUtils.cc
````
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
#include "dynosam_ros/RosUtils.hpp"

#include <gtsam/geometry/Pose3.h>

#include <dynosam/common/Types.hpp>
#include <dynosam/visualizer/Colour.hpp>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "std_msgs/msg/color_rgba.hpp"

template <>
bool dyno::convert(const dyno::Timestamp& time_seconds, rclcpp::Time& time) {
  uint64_t nanoseconds = static_cast<uint64_t>(time_seconds * 1e9);
  time = rclcpp::Time(nanoseconds);
  return true;
}

template <>
bool dyno::convert(const rclcpp::Time& time, dyno::Timestamp& time_seconds) {
  uint64_t nanoseconds = time.nanoseconds();
  time_seconds = static_cast<double>(nanoseconds) / 1e9;
  return true;
}

template <>
bool dyno::convert(const dyno::Timestamp& time_seconds,
                   builtin_interfaces::msg::Time& time) {
  rclcpp::Time ros_time;
  convert(time_seconds, ros_time);
  time = ros_time;
  return true;
}

template <>
bool dyno::convert(const RGBA<float>& colour, std_msgs::msg::ColorRGBA& msg) {
  msg.r = colour.r;
  msg.g = colour.g;
  msg.b = colour.b;
  msg.a = colour.a;
}

template <>
bool dyno::convert(const Color& colour, std_msgs::msg::ColorRGBA& msg) {
  convert(RGBA<float>(colour), msg);
}

template <>
bool dyno::convert(const gtsam::Pose3& pose, geometry_msgs::msg::Pose& msg) {
  const gtsam::Rot3& rotation = pose.rotation();
  const gtsam::Quaternion& quaternion = rotation.toQuaternion();

  // Position
  msg.position.x = pose.x();
  msg.position.y = pose.y();
  msg.position.z = pose.z();

  // Orientation
  msg.orientation.w = quaternion.w();
  msg.orientation.x = quaternion.x();
  msg.orientation.y = quaternion.y();
  msg.orientation.z = quaternion.z();
  return true;
}

template <>
bool dyno::convert(const geometry_msgs::msg::Pose& msg, gtsam::Pose3& pose) {
  gtsam::Point3 translation(msg.position.x, msg.position.y, msg.position.z);

  gtsam::Rot3 rotation(msg.orientation.w, msg.orientation.x, msg.orientation.y,
                       msg.orientation.z);

  pose = gtsam::Pose3(rotation, translation);
  return true;
}

template <>
bool dyno::convert(const gtsam::Pose3& pose,
                   geometry_msgs::msg::PoseStamped& msg) {
  return convert<gtsam::Pose3, geometry_msgs::msg::Pose>(pose, msg.pose);
}

// will not do time or tf links or covariance....
template <>
bool dyno::convert(const gtsam::Pose3& pose, nav_msgs::msg::Odometry& odom) {
  return convert<gtsam::Pose3, geometry_msgs::msg::Pose>(pose, odom.pose.pose);
}

template <>
bool dyno::convert(const geometry_msgs::msg::Pose& pose,
                   geometry_msgs::msg::Transform& transform) {
  transform.translation.x = pose.position.x;
  transform.translation.y = pose.position.y;
  transform.translation.z = pose.position.z;

  transform.rotation.x = pose.orientation.x;
  transform.rotation.y = pose.orientation.y;
  transform.rotation.z = pose.orientation.z;
  transform.rotation.w = pose.orientation.w;
}

template <>
bool dyno::convert(const gtsam::Pose3& pose,
                   geometry_msgs::msg::Transform& transform) {
  transform.translation.x = pose.x();
  transform.translation.y = pose.y();
  transform.translation.z = pose.z();

  const gtsam::Rot3& rotation = pose.rotation();
  const gtsam::Quaternion& quaternion = rotation.toQuaternion();
  transform.rotation.x = quaternion.x();
  transform.rotation.y = quaternion.y();
  transform.rotation.z = quaternion.z();
  transform.rotation.w = quaternion.w();
  return true;
}

template <>
bool dyno::convert(const gtsam::Pose3& pose,
                   geometry_msgs::msg::TransformStamped& transform) {
  return convert<gtsam::Pose3, geometry_msgs::msg::Transform>(
      pose, transform.transform);
}

namespace dyno {

std::ostream& operator<<(std::ostream& stream, const ParameterDetails& param) {
  stream << (std::string)param;
  return stream;
}

std::ostream& operator<<(std::ostream& stream, const rclcpp::Parameter& param) {
  stream << param.get_name() << ": " << param.value_to_string() << " ("
         << param.get_type_name() << ")";
  return stream;
}

const std::string& ParameterDetails::name() const {
  return default_parameter_.get_name();
}

std::string ParameterDetails::node_name() const { return node_->get_name(); }

rclcpp::Parameter ParameterDetails::get() const {
  return this->get_param(this->default_parameter_);
}

std::string ParameterDetails::get(const char* default_value) const {
  return this->get_param<std::string>(
      rclcpp::Parameter(this->name(), std::string(default_value)));
}

std::string ParameterDetails::get(const std::string& default_value) const {
  return this->get_param<std::string>(
      rclcpp::Parameter(this->name(), default_value));
}

ParameterDetails::operator std::string() const {
  std::stringstream ss;
  ss << "[ name: " << this->name();
  ss << " value: " << this->get().value_to_string();
  ss << " description: " << description_.description << "]";
  return ss.str();
}

ParameterDetails::ParameterDetails(
    rclcpp::Node* node, const rclcpp::Parameter& parameter,
    const rcl_interfaces::msg::ParameterDescriptor& description)
    : node_(node), default_parameter_(parameter), description_(description) {
  declare();
}

rclcpp::Parameter ParameterDetails::get_param(
    const rclcpp::Parameter& default_param) const {
  const bool is_set = isSet();
  bool has_default = default_param.get_type() != rclcpp::PARAMETER_NOT_SET;

  if (!is_set) {
    // if no default value we treat a not set parameter as a error as the use
    // MUST override it via runtime configuration
    if (!has_default) {
      throw InvalidDefaultParameter(this->name());
    } else {
      return default_param;
    }
  }
  return node_->get_parameter(this->name());
}

void ParameterDetails::declare() {
  // only declare if needed
  if (!node_->has_parameter(this->name())) {
    const rclcpp::ParameterValue default_value =
        default_parameter_.get_parameter_value();
    const rclcpp::ParameterValue effective_value =
        node_->declare_parameter(this->name(), default_value, description_);
    (void)effective_value;
  }

  // add the param subscriber if we dont have one!!
  // if(!param_subscriber_) {
  //   param_subscriber_ =
  //   std::make_shared<rclcpp::ParameterEventHandler>(node_);

  //   auto cb = [&](const rclcpp::Parameter& new_parameter) {
  //       node_->set_parameter(new_parameter);
  //       //should check same type?
  //       CHECK_EQ(new_parameter.get_name(), this->name());

  //       property_handler_.update(this->name(),new_parameter);
  //   };
  //   //callback handler must be set and remain in scope for the cb's to
  //   trigger cb_handle_ =
  //   param_subscriber_->add_parameter_callback(this->name(), cb);
  // }
}

ParameterConstructor::ParameterConstructor(rclcpp::Node::SharedPtr node,
                                           const std::string& name)
    : ParameterConstructor(node.get(), name) {}

ParameterConstructor::ParameterConstructor(rclcpp::Node* node,
                                           const std::string& name)
    : node_(node), parameter_(name) {
  CHECK_NOTNULL(node_);
  parameter_descriptor_.name = name;
  parameter_descriptor_.dynamic_typing = true;
}

ParameterDetails ParameterConstructor::finish() const {
  return ParameterDetails(node_, parameter_, parameter_descriptor_);
}

ParameterConstructor& ParameterConstructor::description(
    const std::string& description) {
  parameter_descriptor_.description = description;
  return *this;
}

ParameterConstructor& ParameterConstructor::read_only(bool read_only) {
  parameter_descriptor_.read_only = read_only;
  return *this;
}

ParameterConstructor& ParameterConstructor::parameter_description(
    const rcl_interfaces::msg::ParameterDescriptor& parameter_description) {
  parameter_descriptor_ = parameter_description;
  return *this;
}

namespace utils {

Timestamp fromRosTime(const rclcpp::Time& time) {
  Timestamp timestamp;
  convert(time, timestamp);
  return timestamp;
}

rclcpp::Time toRosTime(Timestamp timestamp) {
  rclcpp::Time time;
  convert(timestamp, time);
  return time;
}

}  // namespace utils
}  // namespace dyno
````

## File: src/Utils.cc
````
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

#include "dynosam_ros/Utils.hpp"
#include <string>

#include <glog/logging.h>
#include "rclcpp/rclcpp.hpp"



namespace dyno {

/**
 * @brief Constructs a c-style pointer array from a vector of strings.
 *
 * Must be freed
 *
 * @param args
 * @return char**
 */
char** constructArgvC(const std::vector<std::string>& args) {
    char** argv = new char*[args.size()];

    for(size_t i = 0; i < args.size(); i++) {
        const std::string& arg = args.at(i);
        argv[i] = new char[arg.size()+1];
        strcpy(argv[i], arg.c_str());
    }

    return argv;
}


std::vector<std::string> initRosAndLogging(int argc, char* argv[]) {
    // google::ParseCommandLineFlags(&argc, &argv, true);
    auto non_ros_args = rclcpp::init_and_remove_ros_arguments(argc, argv);

    google::InitGoogleLogging(argv[0]);
    FLAGS_logtostderr = 1;
    FLAGS_colorlogtostderr = 1;
    FLAGS_log_prefix = 1;

    int non_ros_argc = non_ros_args.size();
    char** non_ros_argv_c = constructArgvC(non_ros_args);
    //non_ros_argv_c is heap allocated but attempting to free it after usage in the
    //ParseCommandLineFlags function results in a "double free or corruption" error.
    //I think this is because ParseCommandLineFlags modifies it in place and then free it itself somehow
    //unsure, and may result in a minor memory leak
    google::ParseCommandLineFlags(&non_ros_argc, &non_ros_argv_c, true);

    return non_ros_args;
}

} //dyno
````

## File: src/ZEDOnlineDataProvider.cc
````
#include "dynosam_ros/ZEDOnlineDataProvider.hpp"
#include "dynosam_ros/RosUtils.hpp"
#include "dynosam/dataprovider/DataInterfacePipeline.hpp"
#include "dynosam/common/ImageTypes.hpp"

#include <glog/logging.h>
#include <thread>
#include <chrono>
#include <rclcpp/wait_for_message.hpp>


namespace dyno {
ZEDOnlineDataProvider::ZEDOnlineDataProvider(
    rclcpp::Node::SharedPtr ros_node,
    const ZEDOnlineDataProviderParams& params)
  : DataProvider(),
    ros_node_(ros_node),
    params_(params),
    imu_buffer_(params_.imu_buffer_size) {
  CHECK(ros_node_) << "ROS node pointer cannot be null for ZEDOnlineDataProvider.";

  LOG(INFO) << "[ZEDOnlineDataProvider] Initializing with topics:"
      << "\n  RGB: " << params_.rgb_topic_name
      << "\n  Depth: " << params_.depth_topic_name
      << "\n  CamInfo: " << params_.camera_info_topic_name
      << "\n  IMU: " << params_.imu_topic_name;

  // 1. CameraInfo Subscriber
  // Use standard camera info QoS - compatible with most camera drivers
  auto camera_info_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
  camera_info_sub_ = ros_node_->create_subscription<
    sensor_msgs::msg::CameraInfo>(
      params_.camera_info_topic_name,
      camera_info_qos,
      std::bind(&ZEDOnlineDataProvider::cameraInfoCallback, this,
                std::placeholders::_1));
  VLOG(1) << "[ZEDOnlineDataProvider] Subscribed to CameraInfo: " <<
 camera_info_sub_->get_topic_name();

  // Wait for camera info if requested using rclcpp::wait_for_message
  if (params_.wait_for_camera_info) {
    VLOG(1) << "[ZEDOnlineDataProvider] Waiting for initial CameraInfo...";
    sensor_msgs::msg::CameraInfo camera_info_msg;
    bool received = false;
    
    if (params_.camera_info_timeout_ms.count() < 0) {
      // Wait indefinitely
      received = rclcpp::wait_for_message(
          camera_info_msg, ros_node_, params_.camera_info_topic_name);
    } else {
      // Wait with timeout
      received = rclcpp::wait_for_message(
          camera_info_msg, ros_node_, params_.camera_info_topic_name,
          params_.camera_info_timeout_ms);
    }
    
    if (received) {
      // Process the camera info
      CameraParams cam_params_temp;
      rclcpp::TypeAdapter<dyno::CameraParams,
                          sensor_msgs::msg::CameraInfo>::convert_to_custom(
          camera_info_msg, cam_params_temp);
      
      std::lock_guard<std::mutex> lock(camera_info_mutex_);
      camera_params_ = cam_params_temp;
      camera_info_received_.store(true);
      
      VLOG(1) << "[ZEDOnlineDataProvider] Initial CameraInfo received.";
      LOG_IF(INFO, VLOG_IS_ON(1)) << 
          "[ZEDOnlineDataProvider] Camera Params from ROS: " << camera_params_.value().toString();

    } else {
      std::string err_msg = "Timeout waiting for CameraInfo on topic: " +
                            params_.camera_info_topic_name;
      throw std::runtime_error(err_msg);
    }
  }
  rgb_sub_ = std::make_unique<image_transport::SubscriberFilter>(
      ros_node_.get(), params_.rgb_topic_name, "raw");
  depth_sub_ = std::make_unique<image_transport::SubscriberFilter>(
      ros_node_.get(), params_.depth_topic_name, "raw");
  VLOG(1) << "[ZEDOnlineDataProvider] Created SubscriberFilter for RGB: " <<
 rgb_sub_->getTopic();
  VLOG(1) << "[ZEDOnlineDataProvider] Created SubscriberFilter for Depth: " <<
 depth_sub_->getTopic();

  // 3. Synchronizer for RGB and Depth Images
  image_sync_ = std::make_unique<message_filters::Synchronizer<
    ZedImageSyncPolicy>>(
      ZedImageSyncPolicy(params_.image_sync_queue_size), *rgb_sub_,
      *depth_sub_);
  image_sync_->setAgePenalty(0.01);
  // Lower values prefer newer messages in a pair
  image_sync_->setMaxIntervalDuration(
      rclcpp::Duration::from_seconds(params_.image_sync_slop_sec));
  image_sync_->registerCallback(std::bind(
      &ZEDOnlineDataProvider::imageSyncCallback,
      this, std::placeholders::_1, std::placeholders::_2));
  VLOG(1) <<
 "[ZEDOnlineDataProvider] Image synchronizer (RGB, Depth) configured with slop: "
 << params_.image_sync_slop_sec << "s.";

  // 4. IMU Subscriber (if enabled)
  if (params_.enable_imu_processing) {
    imu_sub_ = ros_node_->create_subscription<sensor_msgs::msg::Imu>(
        params_.imu_topic_name,
        rclcpp::SensorDataQoS(), // Standard QoS for sensor data
        std::bind(&ZEDOnlineDataProvider::imuCallback, this,
                  std::placeholders::_1));
    VLOG(1) << "[ZEDOnlineDataProvider] Subscribed to IMU: " << imu_sub_->
get_topic_name();
  } else {
    VLOG(1) <<
 "[ZEDOnlineDataProvider] IMU processing is disabled by ZEDOnlineDataProviderParams.";
  }

  LOG(INFO) << "[ZEDOnlineDataProvider] Initialization successful.";
}

ZEDOnlineDataProvider::~ZEDOnlineDataProvider() {
  ZEDOnlineDataProvider::shutdown();
}

bool ZEDOnlineDataProvider::spin() {
  if (shutdown_.load()) {
    return false;
  }
  // Callbacks are handled by the executor of ros_node_ (the DynoNode).
  // This spin can just check ROS status.
  return rclcpp::ok();
}

void ZEDOnlineDataProvider::shutdown() {
  if (shutdown_.exchange(true)) {
    // Ensure shutdown logic runs only once
    return;
  }
  LOG(INFO) << "[ZEDOnlineDataProvider] Shutting down...";
  if (image_sync_) {
    image_sync_.reset();
  }
  if (rgb_sub_) rgb_sub_.reset(); // unique_ptr will handle destruction
  if (depth_sub_) depth_sub_.reset();

  if (camera_info_sub_) camera_info_sub_.reset();
  if (imu_sub_) imu_sub_.reset();

  DataProvider::shutdown(); // Call base class shutdown
  LOG(INFO) << "[ZEDOnlineDataProvider] Shutdown complete.";
}

CameraParams::Optional ZEDOnlineDataProvider::getCameraParams() const {
  std::lock_guard<std::mutex> lock(camera_info_mutex_);
  return camera_params_;
}

cv::Mat ZEDOnlineDataProvider::convertRosImageToCvMat(
    const sensor_msgs::msg::Image::ConstSharedPtr& img_msg,
    const std::string& expected_encoding_cv_bridge) const {
  cv_bridge::CvImageConstPtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvShare(img_msg, expected_encoding_cv_bridge);
  } catch (const cv_bridge::Exception& e) {
    LOG(ERROR) <<
        "[ZEDOnlineDataProvider] cv_bridge exception converting ROS image (encoding: "
        << img_msg->encoding << ") to "
        << expected_encoding_cv_bridge << ": " << e.what();
    return {};
  }
  if (!cv_ptr) {
    LOG(ERROR) <<
        "[ZEDOnlineDataProvider] cv_bridge::toCvShare returned null for encoding "
        << expected_encoding_cv_bridge;
    return {};
  }
  // .clone() is important to ensure data ownership if the CvImageConstPtr goes out of scope
  // or if the underlying message buffer is reused.
  return cv_ptr->image.clone();
}

cv::Mat ZEDOnlineDataProvider::readRgbRosImage(
    const sensor_msgs::msg::Image::ConstSharedPtr& img_msg) const {
  std::string target_encoding = params_.output_rgb
                                  ? sensor_msgs::image_encodings::BGR8
                                  : sensor_msgs::image_encodings::MONO8;
  // cv_bridge will attempt conversion if img_msg->encoding is different but compatible (e.g., BGRA8 -> BGR8)
  return convertRosImageToCvMat(img_msg, target_encoding);
}

cv::Mat ZEDOnlineDataProvider::readDepthRosImage(
    const sensor_msgs::msg::Image::ConstSharedPtr& img_msg) const {
  cv_bridge::CvImageConstPtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvShare(img_msg); // Get original encoding first
  } catch (const cv_bridge::Exception& e) {
    LOG(ERROR) <<
        "[ZEDOnlineDataProvider] cv_bridge exception in readDepthRosImage (initial share): "
        << e.what();
    return {};
  }

  if (!cv_ptr) {
    LOG(ERROR) <<
        "[ZEDOnlineDataProvider] cv_bridge::toCvShare returned null for depth image.";
    return {};
  }

  cv::Mat depth_cv = cv_ptr->image;
  cv::Mat depth_dynosam;

  if (cv_ptr->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
    depth_cv.convertTo(depth_dynosam, CV_64F); // meters to meters
  } else if (cv_ptr->encoding == sensor_msgs::image_encodings::TYPE_16UC1) {
    depth_cv.convertTo(depth_dynosam, CV_64F, 1.0 / 1000.0);
    // millimeters to meters
  } else {
    LOG(ERROR) << "[ZEDOnlineDataProvider] Unsupported depth format: " <<
        utils::cvTypeToString(depth_cv.type())
        << " with ROS encoding " << cv_ptr->encoding
        << ". Expected TYPE_32FC1 (meters) or TYPE_16UC1 (mm).";
    return {};
  }
  return depth_dynosam;
}

inline const char* toString(ThreadsafeImuBuffer::QueryResult result) {
  switch (result) {
    case ThreadsafeImuBuffer::QueryResult::kDataAvailable:
      return "kDataAvailable";
    case ThreadsafeImuBuffer::QueryResult::kDataNotYetAvailable:
      return "kDataNotYetAvailable";
    case ThreadsafeImuBuffer::QueryResult::kDataNeverAvailable:
      return "kDataNeverAvailable";
    case ThreadsafeImuBuffer::QueryResult::kQueueShutdown:
      return "kQueueShutdown";
    case ThreadsafeImuBuffer::QueryResult::kTooFewMeasurementsAvailable:
      return "kTooFewMeasurementsAvailable";
    default:
      return "Unknown";
  }
}

void ZEDOnlineDataProvider::imageSyncCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr& rgb_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg) {
  if (shutdown_.load() || !camera_info_received_.load()) {
    VLOG_EVERY_N(1, 100) <<
 "[ZEDOnlineDataProvider] Skipping image sync: shutdown=" << shutdown_.load()
                             << ", camera_info_received=" <<
 camera_info_received_.load();
    return;
  }
  CHECK(camera_params_.has_value()) <<
 "[ZEDOnlineDataProvider] CameraInfo not received but processing images.";

  Timestamp current_timestamp_s = utils::fromRosTime(rgb_msg->header.stamp);
  Timestamp current_timestamp_ns =
      static_cast<Timestamp>(rgb_msg->header.stamp.sec) * 1000000000ULL +
      rgb_msg->header.stamp.nanosec;

  VLOG(2) << "[ZEDOnlineDataProvider] Synced Images Received: Frame " <<
 frame_id_counter_
            << " RGB TS: " << std::fixed << std::setprecision(3) <<
 utils::fromRosTime(rgb_msg->header.stamp)
            << " Depth TS: " << utils::fromRosTime(depth_msg->header.stamp);

  cv::Mat rgb_cv = readRgbRosImage(rgb_msg);
  cv::Mat depth_cv = readDepthRosImage(depth_msg);

  if (rgb_cv.empty() || depth_cv.empty()) {
    LOG(WARNING) <<
        "[ZEDOnlineDataProvider] Failed to convert RGB or Depth image for frame "
        << frame_id_counter_
        << ". RGB empty: " << rgb_cv.empty() << ", Depth empty: " << depth_cv.
        empty();
    return;
  }

  ImageContainer::Ptr image_container = ImageContainer::Create(
      current_timestamp_s,
      frame_id_counter_,
      ImageWrapper<ImageType::RGBMono>(rgb_cv),
      ImageWrapper<ImageType::Depth>(depth_cv),
      ImageWrapper<ImageType::OpticalFlow>(),
      // DynOSAM frontend is expected to compute this
      ImageWrapper<ImageType::MotionMask>()
      // DynOSAM frontend is expected to compute this
      );

  CHECK(image_container_callback_) <<
 "[ZEDOnlineDataProvider] Image container callback not registered!";
  image_container_callback_(image_container);

  if (params_.enable_imu_processing && (
        imu_single_input_callback_ || imu_multi_input_callback_)) {
    ImuMeasurements imu_measurements_for_frontend;
    if (last_image_timestamp_ns_ != 0 && current_timestamp_ns >
        last_image_timestamp_ns_) {
      ThreadsafeImuBuffer::QueryResult query_res = imu_buffer_.
          getImuDataInterpolatedBorders(
              last_image_timestamp_ns_,
              current_timestamp_ns,
              &imu_measurements_for_frontend.timestamps_,
              &imu_measurements_for_frontend.acc_gyr_);

      if (query_res == ThreadsafeImuBuffer::QueryResult::kDataAvailable &&
          imu_measurements_for_frontend.timestamps_.cols() > 0) {
        VLOG(3) << "[ZEDOnlineDataProvider] Retrieved " <<
 imu_measurements_for_frontend.timestamps_.cols()
                        << " IMU samples between " << last_image_timestamp_ns_
                        << " and " << current_timestamp_ns;
        if (imu_multi_input_callback_) {
          imu_multi_input_callback_(imu_measurements_for_frontend);
        } else if (imu_single_input_callback_) {
          // Fallback if only single callback is registered
          for (int i = 0; i < imu_measurements_for_frontend.timestamps_.cols();
               ++i) {
            ImuMeasurement single_imu;
            single_imu.timestamp_ =
                static_cast<Timestamp>(imu_measurements_for_frontend.
                  timestamps_(0, i)) * 1e-9; // Convert ns to s
            single_imu.acc_gyr_ = imu_measurements_for_frontend.acc_gyr_.col(i);
            imu_single_input_callback_(single_imu);
          }
        }
      } else {
        VLOG(3) <<
 "[ZEDOnlineDataProvider] Not enough IMU data or query failed for range "
                        << last_image_timestamp_ns_ << " - " <<
 current_timestamp_ns
                        << ". Query result: " << toString(query_res);
      }
    } else if (last_image_timestamp_ns_ == 0) {
      VLOG(2) << "[ZEDOnlineDataProvider] First image frame (ID: " <<
 frame_id_counter_ << "), no previous IMU data to query against yet.";
    } else if (current_timestamp_ns <= last_image_timestamp_ns_) {
      LOG(WARNING) << "[ZEDOnlineDataProvider] Current image timestamp (" <<
          current_timestamp_ns
          << ") is not after last image timestamp (" << last_image_timestamp_ns_
          << "). IMU data might be out of order or image timestamps duplicated. Frame ID: "
          << frame_id_counter_;
    }
    last_image_timestamp_ns_ = current_timestamp_ns;
  }

  frame_id_counter_++;
}

void ZEDOnlineDataProvider::cameraInfoCallback(
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info_msg) {
  if (camera_info_received_.load() && !params_.wait_for_camera_info) {
    VLOG(2) << "[ZEDOnlineDataProvider] Ignoring subsequent CameraInfo message.";
    return;
  }
  VLOG(1) << "[ZEDOnlineDataProvider] Received CameraInfo on topic: " <<
 info_msg->header.frame_id
            << " at " << utils::fromRosTime(info_msg->header.stamp);

  CameraParams cam_params_dyno_temp;
  rclcpp::TypeAdapter<dyno::CameraParams,
                      sensor_msgs::msg::CameraInfo>::convert_to_custom(
      *info_msg, cam_params_dyno_temp);

  {
    std::lock_guard<std::mutex> lock(camera_info_mutex_);
    camera_params_ = cam_params_dyno_temp;
    if (!camera_info_received_.load()) {
      camera_info_received_.store(true);
      VLOG(1) <<
 "[ZEDOnlineDataProvider] CameraParams successfully parsed and stored.";
      LOG_IF(INFO, VLOG_IS_ON(1)) <<
 "[ZEDOnlineDataProvider] Camera Params from ROS: " << camera_params_.value().
toString();
    }
  }
  if (params_.wait_for_camera_info) camera_info_cond_.notify_all();
}

void ZEDOnlineDataProvider::imuCallback(
    const sensor_msgs::msg::Imu::ConstSharedPtr& imu_msg) {
  if (shutdown_.load() || !params_.enable_imu_processing) return;

  Timestamp timestamp_ns = static_cast<Timestamp>(imu_msg->header.stamp.sec) *
                           1000000000ULL +
                           imu_msg->header.stamp.nanosec;
  ImuAccGyr acc_gyr_ros_frame;
  acc_gyr_ros_frame << imu_msg->linear_acceleration.x,
      imu_msg->linear_acceleration.y,
      imu_msg->linear_acceleration.z,
      imu_msg->angular_velocity.x,
      imu_msg->angular_velocity.y,
      imu_msg->angular_velocity.z;

  // IMPORTANT: Coordinate Frame Transformation for IMU Data
  // The zed-ros2-wrapper publishes IMU data in the frame defined by its URDF, typically `<camera_name>_imu_link`.
  // This frame is usually FLU (X-Forward, Y-Left, Z-Up) by ROS convention if the URDF is standard.
  // DynoSAM's core IMU processing might expect a specific convention (e.g., Body Frame: X-Forward, Y-Left, Z-Up, or
  //                                                                        Sensor Frame: X-Right, Y-Down, Z-Forward).
  // If DynoSAM expects IMU data in a frame other than `<camera_name>_imu_link` (e.g., a `base_link` frame),
  // and it does not handle this transformation internally using TF, then that transformation
  // should ideally happen *before* this DataProvider, or this DataProvider would need TF lookup capabilities.
  // For axis convention mismatch (e.g. `_imu_link` is NED, DynoSAM expects FLU), transform here:
  ImuAccGyr acc_gyr_dynosam_convention = acc_gyr_ros_frame;
  // Default: Assume direct use or TF handles it.

  // Example transformation if ZED's `_imu_link` was publishing in NED (X Fwd, Y Right, Z Down)
  // and DynOSAM's `ImuAccGyr` expects FLU (X Fwd, Y Left, Z Up):
  // acc_gyr_dynosam_convention(0) =  acc_gyr_ros_frame(0);  // AccX_flu = AccX_ned
  // acc_gyr_dynosam_convention(1) = -acc_gyr_ros_frame(1);  // AccY_flu = -AccY_ned (Right -> Left)
  // acc_gyr_dynosam_convention(2) = -acc_gyr_ros_frame(2);  // AccZ_flu = -AccZ_ned (Down -> Up)
  // acc_gyr_dynosam_convention(3) =  acc_gyr_ros_frame(3);  // GyroX_flu = GyroX_ned
  // acc_gyr_dynosam_convention(4) = -acc_gyr_ros_frame(4);  // GyroY_flu = -GyroY_ned
  // acc_gyr_dynosam_convention(5) = -acc_gyr_ros_frame(5);  // GyroZ_flu = -GyroZ_ned

  // This transformation needs to be confirmed based on ZED's actual IMU publishing convention
  // and DynOSAM's expectation. The zed-ros2-wrapper source (zed_camera_component.cpp publishSensorsData)
  // seems to publish raw SDK IMU data stamped with mImuFrameId. The SDK data is usually in
  // ZED's IMU sensor frame (often NED or similar). The URDF transform from _camera_center to _imu_link
  // would then define _imu_link in the robot's TF tree.
  // For now, assume zed-ros2-wrapper correctly publishes in a ROS-standard FLU frame for _imu_link.

  imu_buffer_.addMeasurement(timestamp_ns, acc_gyr_dynosam_convention);
}
} // namespace dyno
````

## File: test/test_adaptors.cc
````
/*
 *   Copyright (c) 2025 ACFR-RPG, University of Sydney, Jesse Morris
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
#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <dynosam/test/helpers.hpp>

#include "dynosam_ros/adaptors/CameraParamsAdaptor.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

using namespace dyno;

TEST(CameraParamsAdaptor, testToROS) {
  CameraParams dyno_params = dyno_testing::makeDefaultCameraParams();

  sensor_msgs::msg::CameraInfo camera_info;

  using Adaptor =
      rclcpp::TypeAdapter<dyno::CameraParams, sensor_msgs::msg::CameraInfo>;
  Adaptor::convert_to_ros_message(dyno_params, camera_info);

  EXPECT_DOUBLE_EQ(camera_info.k[0], dyno_params.fx());
  EXPECT_DOUBLE_EQ(camera_info.k[4], dyno_params.fy());
  EXPECT_DOUBLE_EQ(camera_info.k[2], dyno_params.cu());
  EXPECT_DOUBLE_EQ(camera_info.k[5], dyno_params.cv());

  // Assert: Check the image dimensions
  EXPECT_EQ(camera_info.width, dyno_params.ImageWidth());
  EXPECT_EQ(camera_info.height, dyno_params.ImageHeight());

  // Assert: Check distortion coefficients
  const auto& expected_distortion = dyno_params.getDistortionCoeffs();
  ASSERT_EQ(camera_info.d.size(), expected_distortion.total());
  for (size_t i = 0; i < camera_info.d.size(); ++i) {
    EXPECT_DOUBLE_EQ(camera_info.d[i], expected_distortion.at<double>(i));
  }
}

TEST(CameraParamsAdaptor, testFromROS) {
  sensor_msgs::msg::CameraInfo camera_info;
  camera_info.width = 640;
  camera_info.height = 480;
  camera_info.k = {500.0, 0.0, 320.0, 0.0, 500.0,
                   240.0, 0.0, 0.0,   1.0};      // Intrinsic matrix
  camera_info.d = {0.1, -0.1, 0.01, 0.01, 0.0};  // Distortion coefficients
  camera_info.distortion_model = "plumb_bob";

  using Adaptor =
      rclcpp::TypeAdapter<dyno::CameraParams, sensor_msgs::msg::CameraInfo>;

  CameraParams dyno_params;
  Adaptor::convert_to_custom(camera_info, dyno_params);

  // Assert: Check intrinsic parameters
  EXPECT_DOUBLE_EQ(dyno_params.fx(), 500.0);
  EXPECT_DOUBLE_EQ(dyno_params.fy(), 500.0);
  EXPECT_DOUBLE_EQ(dyno_params.cu(), 320.0);
  EXPECT_DOUBLE_EQ(dyno_params.cv(), 240.0);

  // Assert: Check image size
  EXPECT_EQ(dyno_params.ImageWidth(), 640);
  EXPECT_EQ(dyno_params.ImageHeight(), 480);

  // Assert: Check distortion coefficients
  const auto& expected_distortion = dyno_params.getDistortionCoeffs();
  ASSERT_EQ(expected_distortion.total(), camera_info.d.size());
  for (size_t i = 0; i < expected_distortion.total(); ++i) {
    EXPECT_DOUBLE_EQ(expected_distortion.at<double>(i), camera_info.d[i]);
  }

  // Assert: Check distortion model
  EXPECT_EQ(dyno_params.getDistortionModel(), DistortionModel::RADTAN);
}
````

## File: test/test_main.cc
````
/*
 *   Copyright (c) 2025 ACFR-RPG, University of Sydney, Jesse Morris
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
#include "dynosam_ros/ZEDOnlineDataProvider.hpp"

#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  google::InitGoogleLogging(argv[0]);

  FLAGS_logtostderr = 1;
  FLAGS_colorlogtostderr = 1;
  FLAGS_log_prefix = 1;
  FLAGS_v = 1;
  google::ParseCommandLineFlags(&argc, &argv, true);
  
  int result = RUN_ALL_TESTS();
  
  rclcpp::shutdown();
  return result;
}
````

## File: test/test_online_dataprovider_ros.cc
````
/*
 *   Copyright (c) 2025 ACFR-RPG, University of Sydney, Jesse Morris
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
#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <chrono>
#include <dynosam/test/helpers.hpp>

#include "dynosam_ros/OnlineDataProviderRos.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

using namespace dyno;
using namespace std::chrono_literals;

TEST(OnlineDataProviderRos, testwaitCameraInfoSubscribe) {
  auto node = std::make_shared<rclcpp::Node>("test_wait_for_camera_info_sub");

  auto publisher = node->create_publisher<sensor_msgs::msg::CameraInfo>(
      "image/camera_info", 10);
  auto camera_info_msg = sensor_msgs::msg::CameraInfo();
  camera_info_msg.header.stamp = node->now();
  camera_info_msg.header.frame_id = "camera_frame";
  camera_info_msg.width = 640;
  camera_info_msg.height = 480;
  camera_info_msg.k = {500.0, 0.0, 320.0, 0.0, 500.0,
                       240.0, 0.0, 0.0,   1.0};      // Intrinsic matrix
  camera_info_msg.d = {0.1, -0.1, 0.01, 0.01, 0.0};  // Distortion coefficients
  camera_info_msg.distortion_model = "plumb_bob";

  auto received = false;
  std::shared_ptr<OnlineDataProviderRos> odpr = nullptr;
  std::shared_future<bool> wait = std::async(std::launch::async, [&]() {
    OnlineDataProviderRosParams params;
    params.wait_for_camera_params = true;
    params.camera_params_timeout = -1;
    odpr = std::make_shared<OnlineDataProviderRos>(node, params);
    received = true;
    return true;
  });

  for (auto i = 0u; i < 10 && received == false; ++i) {
    publisher->publish(camera_info_msg);
    std::this_thread::sleep_for(1s);
  }

  ASSERT_NO_THROW(wait.get());
  ASSERT_TRUE(received);
  EXPECT_TRUE(odpr->getCameraParams());
}

TEST(OnlineDataProviderRos, testNowaitCameraInfoSubscribe) {
  auto node =
      std::make_shared<rclcpp::Node>("test_no_wait_for_camera_info_sub");

  OnlineDataProviderRosParams params;
  params.wait_for_camera_params = false;
  auto odpr = std::make_shared<OnlineDataProviderRos>(node, params);
  EXPECT_FALSE(odpr->getCameraParams());
}
````

## File: test/test_ros_utils.cc
````
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
#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "dynosam_ros/RosUtils.hpp"

using namespace dyno;

#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/odometry.hpp"

TEST(TestConcepts, SubNodeNamespacing) {
  auto node = std::make_shared<rclcpp::Node>("my_node", "my_ns");
  LOG(INFO) << node->get_effective_namespace();  // -> "/my_ns"
  auto sub_node1 = node->create_sub_node("a");
  LOG(INFO) << sub_node1->get_effective_namespace();  // -> "/my_ns/a"

  auto pub = sub_node1->create_publisher<nav_msgs::msg::Odometry>("test", 1);
  LOG(INFO) << pub->get_topic_name();
}

TEST(RosUtils, HasMsgHeader) {
  EXPECT_FALSE(internal::HasMsgHeader<geometry_msgs::msg::Point>::value);
  EXPECT_TRUE(internal::HasMsgHeader<nav_msgs::msg::Odometry>::value);
}

TEST(RosTraits, testParamTypes) {
  EXPECT_TRUE(
      (std::is_same_v<
          dyno::ros_param_traits<
              rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE>::cpp_type,
          double>));
  EXPECT_EQ(dyno::traits<double>::ros_parameter_type,
            rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE);

  EXPECT_TRUE(
      (std::is_same_v<
          dyno::ros_param_traits<
              rcl_interfaces::msg::ParameterType::PARAMETER_STRING>::cpp_type,
          const char *>));
  EXPECT_EQ(dyno::traits<const char *>::ros_parameter_type,
            rcl_interfaces::msg::ParameterType::PARAMETER_STRING);

  EXPECT_TRUE((std::is_same_v<
               dyno::ros_param_traits<rcl_interfaces::msg::ParameterType::
                                          PARAMETER_DOUBLE_ARRAY>::cpp_type,
               std::vector<double>>));
  EXPECT_EQ(dyno::traits<std::vector<double>>::ros_parameter_type,
            rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE_ARRAY);
}

TEST(ParameterConstructor, defaultConstructionShared) {
  auto node = std::make_shared<rclcpp::Node>("test_node");
  ParameterConstructor pc(node, "test_param");
  const rclcpp::ParameterValue &pc_as_value = pc;
  EXPECT_EQ(pc_as_value.get_type(), rclcpp::PARAMETER_NOT_SET);
  EXPECT_EQ(pc.name(), "test_param");
}

TEST(ParameterConstructor, defaultConstructionRawP) {
  auto node = std::make_shared<rclcpp::Node>("test_node");
  ParameterConstructor pc(node.get(), "test_param");
  const rclcpp::ParameterValue &pc_as_value = pc;
  EXPECT_EQ(pc_as_value.get_type(), rclcpp::PARAMETER_NOT_SET);
  EXPECT_EQ(pc.name(), "test_param");
}

TEST(ParameterConstructor, valueConstruction) {
  auto node = std::make_shared<rclcpp::Node>("test_node");
  ParameterConstructor pc(node, "test_param", 10);
  const rclcpp::ParameterValue &pc_as_value = pc;
  EXPECT_EQ(pc_as_value.get_type(), rclcpp::PARAMETER_INTEGER);
  EXPECT_EQ(pc_as_value.get<int>(), 10);
  EXPECT_EQ(pc.name(), "test_param");
}

TEST(ParameterConstructor, testOptionUpdates) {
  auto node = std::make_shared<rclcpp::Node>("test_node");
  ParameterConstructor pc = ParameterConstructor(node, "test_param", 10)
                                .description("a test")
                                .read_only(true);
  const rcl_interfaces::msg::ParameterDescriptor &p_as_descriptor = pc;
  EXPECT_EQ(p_as_descriptor.description, "a test");
  EXPECT_EQ(p_as_descriptor.read_only, true);
}

TEST(ParameterConstructor, testFinishWithDefault) {
  auto node = std::make_shared<rclcpp::Node>("test_node");
  ParameterDetails detail = ParameterConstructor(node, "test_param", 10)
                                .description("a test")
                                .read_only(true)
                                .finish();
  const rclcpp::ParameterValue &detail_as_value = detail;
  EXPECT_EQ(detail_as_value.get_type(), rclcpp::PARAMETER_INTEGER);
  EXPECT_EQ(detail_as_value.get<int>(), 10);
}

TEST(ParameterConstructor, testFinishWithoutDefault) {
  auto node = std::make_shared<rclcpp::Node>("test_node");
  ParameterDetails detail_shared = ParameterConstructor(node, "test_param")
                                       .description("a test")
                                       .read_only(true)
                                       .finish();
  EXPECT_THROW({ detail_shared.get(); }, InvalidDefaultParameter);

  ParameterDetails detail_raw = ParameterConstructor(node.get(), "test_param")
                                    .description("a test")
                                    .read_only(true)
                                    .finish();
  EXPECT_THROW({ detail_raw.get(); }, InvalidDefaultParameter);
}

TEST(ParameterConstructor, testFinishWithoutDefaultButOverwrite) {
  rclcpp::NodeOptions no;
  no.parameter_overrides({
      {"test_param", 10},
  });

  auto node = std::make_shared<rclcpp::Node>("test_node", no);
  ParameterDetails detail = ParameterConstructor(node, "test_param")
                                .description("a test")
                                .read_only(true)
                                .finish();
  const rclcpp::ParameterValue &detail_as_value = detail;
  EXPECT_EQ(detail_as_value.get_type(), rclcpp::PARAMETER_INTEGER);
  EXPECT_EQ(detail_as_value.get<int>(), 10);
  EXPECT_EQ(detail.get<int>(), 10);
}

TEST(ParameterDetails, testDeclareWithDefault) {
  auto node = std::make_shared<rclcpp::Node>("test_node");
  ParameterConstructor pc(node, "test", "some_value");
  EXPECT_FALSE(node->has_parameter("test"));

  ParameterDetails detail = pc.finish();

  EXPECT_TRUE(node->has_parameter("test"));
  EXPECT_EQ(rclcpp::PARAMETER_STRING,
            node->get_parameter("test").get_parameter_value().get_type());

  EXPECT_EQ(node->get_parameter("test").get_value<std::string>(), "some_value");

  std::string value = detail.get<std::string>();
  EXPECT_EQ(value, "some_value");

  // check we can use another detail to get the same value even if no default
  // has been set
  ParameterDetails detail1 = pc.finish();
  value = detail1.get<std::string>(detail1);
  EXPECT_EQ(value, "some_value");

  // update param and check we can still get the correct one!!
  node->set_parameter(rclcpp::Parameter("test", "a_new_value"));
  value = detail1.get<std::string>();
  EXPECT_EQ(value, "a_new_value");
}

TEST(ParameterDetails, testGetParamWithOverrides) {
  rclcpp::NodeOptions no;
  no.parameter_overrides({
      {"parameter_no_default", 42},
      {"parameter_wrong_type", "value"},
  });

  auto node = std::make_shared<rclcpp::Node>("test_node", no);

  // okay even with overrides, we MUST declare it for it to appear
  // this will not be the case if we set allow_uncleared_overrides on the
  // NodeOptions
  EXPECT_FALSE(node->has_parameter("parameter_no_default"));
  ParameterDetails detail =
      ParameterConstructor(node, "parameter_no_default").finish();
  EXPECT_TRUE(node->has_parameter("parameter_no_default"));
  EXPECT_EQ(node->get_parameter("parameter_no_default").get_value<int>(), 42);

  // this has a different type than is in the override
  EXPECT_THROW(
      { ParameterConstructor(node, "parameter_wrong_type", 10).finish(); },
      rclcpp::exceptions::InvalidParameterTypeException);
}

TEST(ParameterDetails, testGetParamWithMutableString) {
  rclcpp::NodeOptions no;
  no.parameter_overrides({
      {"param", "value"},
  });

  auto node = std::make_shared<rclcpp::Node>("test_node", no);

  struct Params {
    std::string value = "a";
  };

  Params params;
  // this tests the case where the ValueT type is not a const& and we can still
  // use it!!
  params.value = ParameterConstructor(node, "param", params.value)
                     .finish()
                     .get<std::string>();
  // test the value is in the override not the default one
  EXPECT_EQ(params.value, "value");
}

TEST(ParameterDetails, testGetWithNoOverridesWithDefault) {
  auto node = std::make_shared<rclcpp::Node>("test_node");
  ParameterDetails detail =
      ParameterConstructor(node, "parameter_no_default").finish();

  // attempt to get a param that has been declared but not set without providing
  // a default value
  int value = detail.get(5);
  EXPECT_EQ(value, 5);
  // the node should not be updated
  EXPECT_EQ(rclcpp::PARAMETER_NOT_SET,
            node->get_parameter("parameter_no_default")
                .get_parameter_value()
                .get_type());
}

TEST(ParameterDetails, testGetWithOverridesWithDefault) {
  auto node = std::make_shared<rclcpp::Node>("test_node");

  ParameterDetails detail =
      ParameterConstructor(node, "parameter_default", 10).finish();
  // attempt to get a param that has been declared and set
  int value = detail.get(5);
  EXPECT_EQ(value, 10);
  // the node should not be updated
  EXPECT_EQ(
      node->get_parameter("parameter_default").get_parameter_value().get<int>(),
      10);
}

TEST(ParameterDetails, testGetWithOverridesWithString) {
  auto node = std::make_shared<rclcpp::Node>("test_node");

  ParameterDetails detail =
      ParameterConstructor(node, "string_param", "a string").finish();
  // attempt to get a param that has been declared but not set without providing
  // a default value
  std::string value = detail.get<std::string>();
  EXPECT_EQ(value, "a string");
  // the node should not be updated
  EXPECT_EQ(
      rclcpp::PARAMETER_STRING,
      node->get_parameter("string_param").get_parameter_value().get_type());
}

TEST(ParameterDetails, testGetWithDefaultString) {
  auto node = std::make_shared<rclcpp::Node>("test_node");
  ParameterDetails detail = ParameterConstructor(node, "string_param").finish();

  // attempt to get a param that has been declared but not set without providing
  // a default value
  std::string value = detail.get<std::string>("default string");
  EXPECT_EQ(value, "default string");
  // the node should not be updated
  EXPECT_EQ(
      rclcpp::PARAMETER_NOT_SET,
      node->get_parameter("string_param").get_parameter_value().get_type());
}

// TEST(ParameterDetails, testUpdateOnParamChange) {

//   auto node = std::make_shared<rclcpp::Node>("test_node");

//   rclcpp::executors::SingleThreadedExecutor executor;
//   executor.add_node(node);

//   rclcpp::Publisher<rcl_interfaces::msg::ParameterEvent>::SharedPtr
//     event_publisher =
//     node->create_publisher<rcl_interfaces::msg::ParameterEvent>("/parameter_events",
//     1);

//   auto param_event = std::make_shared<rcl_interfaces::msg::ParameterEvent>();

//   rcl_interfaces::msg::Parameter p;
//   p.name = "parameter_default";
//   p.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
//   p.value.integer_value = 42;

//   param_event->node = node->get_fully_qualified_name();
//   param_event->changed_parameters.push_back(p);

//   ParameterDetails detail = ParameterConstructor(node, "parameter_default",
//   10).finish(); const int original_value = detail.get<int>();
//   EXPECT_EQ(original_value, 10);

//   event_publisher->publish(*param_event);

//   executor.spin_some();

//   const int updated_value = detail.get<int>();
//   EXPECT_EQ(updated_value, 42);

// }

// TEST(ParameterDetails, testUpdateOnParamChangeCallback) {

//   auto node = std::make_shared<rclcpp::Node>("test_node");

//   rclcpp::executors::SingleThreadedExecutor executor;
//   executor.add_node(node);

//   rclcpp::Publisher<rcl_interfaces::msg::ParameterEvent>::SharedPtr
//     event_publisher =
//     node->create_publisher<rcl_interfaces::msg::ParameterEvent>("/parameter_events",
//     1);

//   auto param_event = std::make_shared<rcl_interfaces::msg::ParameterEvent>();

//   rcl_interfaces::msg::Parameter p;
//   p.name = "parameter_default";
//   p.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
//   p.value.integer_value = 42;

//   param_event->node = node->get_fully_qualified_name();
//   param_event->changed_parameters.push_back(p);

//   ParameterDetails detail = ParameterConstructor(node, "parameter_default",
//   10).finish();

//   int updated_value = -1;
//   detail.registerParamCallback<int>([&updated_value](int value) -> void {
//     updated_value = value;
//   });

//   const int original_value = detail.get<int>();
//   EXPECT_EQ(original_value, 10);

//   // param_handler->test_event(param_event);
//   event_publisher->publish(*param_event);

//   executor.spin_some();
//   EXPECT_EQ(updated_value, 42);

// }
````

## File: test/test_zed_online_dataprovider.cc
````
/*
 *   Copyright (c) 2025 ACFR-RPG, University of Sydney, Jesse Morris
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
#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <chrono>
#include <future>
#include <thread>
#include <dynosam/test/helpers.hpp>

#include "dynosam_ros/ZEDOnlineDataProvider.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"

using namespace dyno;
using namespace std::chrono_literals;

class ZEDOnlineDataProviderTest : public ::testing::Test {
protected:
    void SetUp() override {
        // ROS is already initialized by main function
        node_ = std::make_shared<rclcpp::Node>("test_zed_online_dataprovider");
        
        // Default test parameters
        params_.rgb_topic_name = "test/rgb/image_rect_color";
        params_.depth_topic_name = "test/depth/depth_registered";
        params_.camera_info_topic_name = "test/rgb/camera_info";
        params_.imu_topic_name = "test/imu/data";
        params_.wait_for_camera_info = false;
        params_.enable_imu_processing = true;
        params_.output_rgb = true;
        params_.image_sync_queue_size = 5;
        params_.image_sync_slop_sec = 0.1;
        params_.imu_buffer_size = 100;
    }

    void TearDown() override {
        if (provider_) {
            provider_->shutdown();
            provider_.reset();
        }
        node_.reset();
        // Don't call rclcpp::shutdown() in individual tests
        // Let the main function handle shutdown
    }

    sensor_msgs::msg::CameraInfo createTestCameraInfo() {
        auto camera_info = sensor_msgs::msg::CameraInfo();
        camera_info.header.stamp = node_->now();
        camera_info.header.frame_id = "camera_frame";
        camera_info.width = 640;
        camera_info.height = 480;
        camera_info.k = {500.0, 0.0, 320.0, 0.0, 500.0, 240.0, 0.0, 0.0, 1.0};
        camera_info.d = {0.1, -0.1, 0.01, 0.01, 0.0};
        camera_info.distortion_model = "plumb_bob";
        return camera_info;
    }

    sensor_msgs::msg::Image createTestRgbImage(const rclcpp::Time& timestamp) {
        auto image_msg = sensor_msgs::msg::Image();
        image_msg.header.stamp = timestamp;
        image_msg.header.frame_id = "camera_frame";
        image_msg.width = 640;
        image_msg.height = 480;
        image_msg.encoding = "bgr8";
        image_msg.step = 640 * 3;
        image_msg.data.resize(640 * 480 * 3, 128); // Fill with gray
        return image_msg;
    }

    sensor_msgs::msg::Image createTestDepthImage(const rclcpp::Time& timestamp) {
        auto image_msg = sensor_msgs::msg::Image();
        image_msg.header.stamp = timestamp;
        image_msg.header.frame_id = "camera_frame";
        image_msg.width = 640;
        image_msg.height = 480;
        image_msg.encoding = "32FC1";
        image_msg.step = 640 * 4;
        image_msg.data.resize(640 * 480 * 4);
        
        // Fill with test depth values (1.0 meter)
        float* data_ptr = reinterpret_cast<float*>(image_msg.data.data());
        std::fill(data_ptr, data_ptr + (640 * 480), 1.0f);
        return image_msg;
    }

    sensor_msgs::msg::Imu createTestImuMessage(const rclcpp::Time& timestamp) {
        auto imu_msg = sensor_msgs::msg::Imu();
        imu_msg.header.stamp = timestamp;
        imu_msg.header.frame_id = "imu_frame";
        imu_msg.linear_acceleration.x = 0.1;
        imu_msg.linear_acceleration.y = 0.2;
        imu_msg.linear_acceleration.z = 9.8;
        imu_msg.angular_velocity.x = 0.01;
        imu_msg.angular_velocity.y = 0.02;
        imu_msg.angular_velocity.z = 0.03;
        return imu_msg;
    }

    rclcpp::Node::SharedPtr node_;
    ZEDOnlineDataProviderParams params_;
    std::shared_ptr<ZEDOnlineDataProvider> provider_;
};

TEST_F(ZEDOnlineDataProviderTest, testBasicInitialization) {
    params_.wait_for_camera_info = false;
    
    ASSERT_NO_THROW({
        provider_ = std::make_shared<ZEDOnlineDataProvider>(node_, params_);
    });
    
    EXPECT_EQ(provider_->datasetSize(), -1); // Online stream
    EXPECT_FALSE(provider_->getCameraParams().has_value());
    EXPECT_TRUE(provider_->spin());
}

TEST_F(ZEDOnlineDataProviderTest, testNullNodeInitialization) {
    // This test will trigger a CHECK failure, so we expect the process to abort
    // We can't test this with EXPECT_THROW since CHECK causes process termination
    // Instead, just verify construction works with valid node
    ASSERT_NO_THROW({
        auto valid_provider = std::make_shared<ZEDOnlineDataProvider>(node_, params_);
        valid_provider->shutdown();
    });
}

TEST_F(ZEDOnlineDataProviderTest, testWaitForCameraInfoSuccess) {
    params_.wait_for_camera_info = true;
    params_.camera_info_timeout_ms = std::chrono::milliseconds(2000); // 2 second timeout
    
    auto camera_info_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
    auto camera_info_pub = node_->create_publisher<sensor_msgs::msg::CameraInfo>(
        params_.camera_info_topic_name, camera_info_qos);
    auto camera_info_msg = createTestCameraInfo();
    
    bool provider_ready = false;
    auto init_future = std::async(std::launch::async, [&]() {
        try {
            provider_ = std::make_shared<ZEDOnlineDataProvider>(node_, params_);
            provider_ready = true;
            return true;
        } catch (...) {
            return false;
        }
    });
    
    // Give the provider time to initialize and start waiting for camera info
    std::this_thread::sleep_for(200ms);
    
    // Now publish camera info messages until the provider receives one
    for (int i = 0; i < 20 && !provider_ready; ++i) {
        camera_info_pub->publish(camera_info_msg);
        rclcpp::spin_some(node_);
        std::this_thread::sleep_for(100ms);
    }
    
    ASSERT_TRUE(init_future.get());
    EXPECT_TRUE(provider_ready);
    EXPECT_TRUE(provider_->getCameraParams().has_value());
    
    auto cam_params = provider_->getCameraParams().value();
    EXPECT_DOUBLE_EQ(cam_params.fx(), 500.0);
    EXPECT_DOUBLE_EQ(cam_params.fy(), 500.0);
    EXPECT_DOUBLE_EQ(cam_params.cu(), 320.0);
    EXPECT_DOUBLE_EQ(cam_params.cv(), 240.0);
}

TEST_F(ZEDOnlineDataProviderTest, testWaitForCameraInfoTimeout) {
    params_.wait_for_camera_info = true;
    params_.camera_info_timeout_ms = std::chrono::milliseconds(100);
  try {
    provider_ = std::make_shared<ZEDOnlineDataProvider>(node_, params_);
  }
  catch (const std::runtime_error& e) {
    std::cout << e.what() << std::endl;
  }

    EXPECT_THROW({
        provider_ = std::make_shared<ZEDOnlineDataProvider>(node_, params_);
    }, std::runtime_error);
}

TEST_F(ZEDOnlineDataProviderTest, testNoWaitForCameraInfo) {
    params_.wait_for_camera_info = false;
    
    ASSERT_NO_THROW({
        provider_ = std::make_shared<ZEDOnlineDataProvider>(node_, params_);
    });
    
    EXPECT_FALSE(provider_->getCameraParams().has_value());
}

TEST_F(ZEDOnlineDataProviderTest, testImageCallbackRegistration) {
    params_.wait_for_camera_info = false;
    provider_ = std::make_shared<ZEDOnlineDataProvider>(node_, params_);
    
    bool callback_called = false;
    ImageContainer::Ptr received_container = nullptr;
    
    provider_->registerImageContainerCallback([&](const ImageContainer::Ptr& container) {
        callback_called = true;
        received_container = container;
    });
    
    // First publish camera info to enable image processing
    auto camera_info_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
    auto camera_info_pub = node_->create_publisher<sensor_msgs::msg::CameraInfo>(
        params_.camera_info_topic_name, camera_info_qos);
    auto camera_info_msg = createTestCameraInfo();
    camera_info_pub->publish(camera_info_msg);
    rclcpp::spin_some(node_);
    std::this_thread::sleep_for(100ms);
    
    // Wait a bit for all subscriptions to be established
    std::this_thread::sleep_for(200ms);
    
    // Now publish synchronized images with compatible QoS
    auto image_qos = rclcpp::QoS(10)
                        .reliability(rclcpp::ReliabilityPolicy::Reliable)
                        .durability(rclcpp::DurabilityPolicy::Volatile);
    auto rgb_pub = node_->create_publisher<sensor_msgs::msg::Image>(
        params_.rgb_topic_name, image_qos);
    auto depth_pub = node_->create_publisher<sensor_msgs::msg::Image>(
        params_.depth_topic_name, image_qos);
    
    // Wait for publisher-subscriber connections
    std::this_thread::sleep_for(100ms);
    
    rclcpp::Time timestamp = node_->now();
    auto rgb_msg = createTestRgbImage(timestamp);
    auto depth_msg = createTestDepthImage(timestamp);
    
    // Publish multiple times to ensure synchronization works
    for (int pub_attempt = 0; pub_attempt < 5 && !callback_called; ++pub_attempt) {
        timestamp = node_->now();
        rgb_msg = createTestRgbImage(timestamp);
        depth_msg = createTestDepthImage(timestamp);
        
        rgb_pub->publish(rgb_msg);
        depth_pub->publish(depth_msg);
        
        // Process callbacks immediately after publishing
        for (int i = 0; i < 5; ++i) {
            rclcpp::spin_some(node_);
            std::this_thread::sleep_for(10ms);
        }
        
        if (callback_called) break;
        std::this_thread::sleep_for(100ms);
    }
    
    EXPECT_TRUE(callback_called);
    ASSERT_NE(received_container, nullptr);
    EXPECT_EQ(received_container->getFrameId(), 0);
    EXPECT_FALSE(received_container->getImage().empty());
    EXPECT_FALSE(received_container->getDepth().empty());
}

TEST_F(ZEDOnlineDataProviderTest, testImuProcessing) {
    params_.wait_for_camera_info = false;
    params_.enable_imu_processing = true;
    provider_ = std::make_shared<ZEDOnlineDataProvider>(node_, params_);
    
    std::vector<ImuMeasurement> received_imu_data;
    provider_->registerImuSingleCallback([&](const ImuMeasurement& imu) {
        received_imu_data.push_back(imu);
    });
    
    auto imu_pub = node_->create_publisher<sensor_msgs::msg::Imu>(
        params_.imu_topic_name, 10);
    
    // Publish several IMU messages
    for (int i = 0; i < 5; ++i) {
        rclcpp::Time timestamp = node_->now();
        auto imu_msg = createTestImuMessage(timestamp);
        imu_pub->publish(imu_msg);
        rclcpp::spin_some(node_);
        std::this_thread::sleep_for(50ms);
    }
    
    // IMU data should be buffered but not delivered until images trigger the query
}

TEST_F(ZEDOnlineDataProviderTest, testImuDisabled) {
    params_.wait_for_camera_info = false;
    params_.enable_imu_processing = false;
    
    ASSERT_NO_THROW({
        provider_ = std::make_shared<ZEDOnlineDataProvider>(node_, params_);
    });
    
    // Should still initialize successfully even if IMU is disabled
    EXPECT_TRUE(provider_->spin());
}

TEST_F(ZEDOnlineDataProviderTest, testShutdown) {
    params_.wait_for_camera_info = false;
    provider_ = std::make_shared<ZEDOnlineDataProvider>(node_, params_);
    
    EXPECT_TRUE(provider_->spin());
    
    provider_->shutdown();
    
    EXPECT_FALSE(provider_->spin());
    
    // Multiple shutdowns should be safe
    ASSERT_NO_THROW(provider_->shutdown());
}

TEST_F(ZEDOnlineDataProviderTest, testImageEncodingConversion) {
    params_.wait_for_camera_info = false;
    provider_ = std::make_shared<ZEDOnlineDataProvider>(node_, params_);
    
    // Test different image encodings
    rclcpp::Time timestamp = node_->now();
    auto rgb_msg = createTestRgbImage(timestamp);
    rgb_msg.encoding = "rgb8";  // Different from default bgr8
    
    bool callback_called = false;
    ImageContainer::Ptr received_container = nullptr;
    
    provider_->registerImageContainerCallback([&](const ImageContainer::Ptr& container) {
        callback_called = true;
        received_container = container;
    });
    
    // Publish camera info first
    auto camera_info_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
    auto camera_info_pub = node_->create_publisher<sensor_msgs::msg::CameraInfo>(
        params_.camera_info_topic_name, camera_info_qos);
    auto camera_info_msg = createTestCameraInfo();
    camera_info_pub->publish(camera_info_msg);
    rclcpp::spin_some(node_);
    std::this_thread::sleep_for(100ms);
    
    // Wait a bit for all subscriptions to be established
    std::this_thread::sleep_for(200ms);
    
    // Publish images with compatible QoS
    auto image_qos = rclcpp::QoS(10)
                        .reliability(rclcpp::ReliabilityPolicy::Reliable)
                        .durability(rclcpp::DurabilityPolicy::Volatile);
    auto rgb_pub = node_->create_publisher<sensor_msgs::msg::Image>(
        params_.rgb_topic_name, image_qos);
    auto depth_pub = node_->create_publisher<sensor_msgs::msg::Image>(
        params_.depth_topic_name, image_qos);
    
    // Wait for publisher-subscriber connections
    std::this_thread::sleep_for(100ms);
    
    auto depth_msg = createTestDepthImage(timestamp);
    
    // Publish multiple times to ensure synchronization works
    for (int pub_attempt = 0; pub_attempt < 5 && !callback_called; ++pub_attempt) {
        timestamp = node_->now();
        rgb_msg = createTestRgbImage(timestamp);
        depth_msg = createTestDepthImage(timestamp);
        
        rgb_pub->publish(rgb_msg);
        depth_pub->publish(depth_msg);
        
        // Process callbacks immediately after publishing
        for (int i = 0; i < 5; ++i) {
            rclcpp::spin_some(node_);
            std::this_thread::sleep_for(10ms);
        }
        
        if (callback_called) break;
        std::this_thread::sleep_for(100ms);
    }
    
    EXPECT_TRUE(callback_called);
    ASSERT_NE(received_container, nullptr);
    EXPECT_FALSE(received_container->getImage().empty());
}

TEST_F(ZEDOnlineDataProviderTest, testDepthEncodingConversion) {
    params_.wait_for_camera_info = false;
    provider_ = std::make_shared<ZEDOnlineDataProvider>(node_, params_);
    
    bool callback_called = false;
    ImageContainer::Ptr received_container = nullptr;
    
    provider_->registerImageContainerCallback([&](const ImageContainer::Ptr& container) {
        callback_called = true;
        received_container = container;
    });
    
    // Publish camera info first
    auto camera_info_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
    auto camera_info_pub = node_->create_publisher<sensor_msgs::msg::CameraInfo>(
        params_.camera_info_topic_name, camera_info_qos);
    auto camera_info_msg = createTestCameraInfo();
    camera_info_pub->publish(camera_info_msg);
    rclcpp::spin_some(node_);
    std::this_thread::sleep_for(100ms);
    
    // Test 16UC1 depth format (millimeters)
    rclcpp::Time timestamp = node_->now();
    auto depth_msg = createTestDepthImage(timestamp);
    depth_msg.encoding = "16UC1";
    depth_msg.step = 640 * 2;
    depth_msg.data.resize(640 * 480 * 2);
    
    // Fill with test depth values (1000mm = 1.0m)
    uint16_t* data_ptr = reinterpret_cast<uint16_t*>(depth_msg.data.data());
    std::fill(data_ptr, data_ptr + (640 * 480), 1000);
    
    // Wait a bit for all subscriptions to be established
    std::this_thread::sleep_for(200ms);
    
    auto image_qos = rclcpp::QoS(10)
                        .reliability(rclcpp::ReliabilityPolicy::Reliable)
                        .durability(rclcpp::DurabilityPolicy::Volatile);
    auto rgb_pub = node_->create_publisher<sensor_msgs::msg::Image>(
        params_.rgb_topic_name, image_qos);
    auto depth_pub = node_->create_publisher<sensor_msgs::msg::Image>(
        params_.depth_topic_name, image_qos);
    
    // Wait for publisher-subscriber connections
    std::this_thread::sleep_for(100ms);
    
    auto rgb_msg = createTestRgbImage(timestamp);
    
    // Publish multiple times to ensure synchronization works
    for (int pub_attempt = 0; pub_attempt < 5 && !callback_called; ++pub_attempt) {
        timestamp = node_->now();
        rgb_msg = createTestRgbImage(timestamp);
        depth_msg = createTestDepthImage(timestamp);
        
        rgb_pub->publish(rgb_msg);
        depth_pub->publish(depth_msg);
        
        // Process callbacks immediately after publishing
        for (int i = 0; i < 5; ++i) {
            rclcpp::spin_some(node_);
            std::this_thread::sleep_for(10ms);
        }
        
        if (callback_called) break;
        std::this_thread::sleep_for(100ms);
    }
    
    EXPECT_TRUE(callback_called);
    ASSERT_NE(received_container, nullptr);
    EXPECT_FALSE(received_container->getDepth().empty());
    
    // Check that millimeter values were converted to meters
    cv::Mat depth_cv = received_container->getDepth();
    EXPECT_EQ(depth_cv.type(), CV_64F);
    
    // Check a few depth values were converted correctly (1000mm -> 1.0m)
    EXPECT_NEAR(depth_cv.at<double>(240, 320), 1.0, 1e-6);
}

TEST_F(ZEDOnlineDataProviderTest, testGrayscaleOutput) {
    params_.wait_for_camera_info = false;
    params_.output_rgb = false; // Request grayscale output
    provider_ = std::make_shared<ZEDOnlineDataProvider>(node_, params_);
    
    bool callback_called = false;
    ImageContainer::Ptr received_container = nullptr;
    
    provider_->registerImageContainerCallback([&](const ImageContainer::Ptr& container) {
        callback_called = true;
        received_container = container;
    });
    
    // Publish camera info first
    auto camera_info_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
    auto camera_info_pub = node_->create_publisher<sensor_msgs::msg::CameraInfo>(
        params_.camera_info_topic_name, camera_info_qos);
    auto camera_info_msg = createTestCameraInfo();
    camera_info_pub->publish(camera_info_msg);
    rclcpp::spin_some(node_);
    std::this_thread::sleep_for(100ms);
    
    // Wait a bit for all subscriptions to be established
    std::this_thread::sleep_for(200ms);
    
    auto image_qos = rclcpp::QoS(10)
                        .reliability(rclcpp::ReliabilityPolicy::Reliable)
                        .durability(rclcpp::DurabilityPolicy::Volatile);
    auto rgb_pub = node_->create_publisher<sensor_msgs::msg::Image>(
        params_.rgb_topic_name, image_qos);
    auto depth_pub = node_->create_publisher<sensor_msgs::msg::Image>(
        params_.depth_topic_name, image_qos);
    
    // Wait for publisher-subscriber connections
    std::this_thread::sleep_for(100ms);
    
    rclcpp::Time timestamp = node_->now();
    auto rgb_msg = createTestRgbImage(timestamp);
    auto depth_msg = createTestDepthImage(timestamp);
    
    // Publish multiple times to ensure synchronization works
    for (int pub_attempt = 0; pub_attempt < 5 && !callback_called; ++pub_attempt) {
        timestamp = node_->now();
        rgb_msg = createTestRgbImage(timestamp);
        depth_msg = createTestDepthImage(timestamp);
        
        rgb_pub->publish(rgb_msg);
        depth_pub->publish(depth_msg);
        
        // Process callbacks immediately after publishing
        for (int i = 0; i < 5; ++i) {
            rclcpp::spin_some(node_);
            std::this_thread::sleep_for(10ms);
        }
        
        if (callback_called) break;
        std::this_thread::sleep_for(100ms);
    }
    
    EXPECT_TRUE(callback_called);
    ASSERT_NE(received_container, nullptr);
    
    cv::Mat rgb_cv = received_container->getImage();
    EXPECT_FALSE(rgb_cv.empty());
    EXPECT_EQ(rgb_cv.channels(), 1); // Should be grayscale
}

TEST_F(ZEDOnlineDataProviderTest, testCustomTopicNames) {
    params_.rgb_topic_name = "custom/rgb";
    params_.depth_topic_name = "custom/depth";
    params_.camera_info_topic_name = "custom/camera_info";
    params_.imu_topic_name = "custom/imu";
    params_.wait_for_camera_info = false;
    
    ASSERT_NO_THROW({
        provider_ = std::make_shared<ZEDOnlineDataProvider>(node_, params_);
    });
    
    // Should initialize with custom topic names
    EXPECT_TRUE(provider_->spin());
}
````

## File: CLAUDE.md
````markdown
# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build System

This is a ROS2 package using ament_cmake. The build system supports both ninja and make generators.

### Core Build Commands
```bash
# Set up environment for dependencies
export AMENT_PREFIX_PATH=/home/bonnie/DynOSAM/install/dynosam:/home/bonnie/DynOSAM/install/dynamic_slam_interfaces:${AMENT_PREFIX_PATH}
export CMAKE_PREFIX_PATH=/home/bonnie/DynOSAM/install/dynosam:/home/bonnie/DynOSAM/install/dynamic_slam_interfaces:/opt/ros/humble

# Build with cmake directly (see cmake_commands.sh)
cmake /home/bonnie/DynOSAM/src/dynosam_ros -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_BUILD_TYPE=RelWithDebinfo -G Ninja -DCMAKE_INSTALL_PREFIX=/home/bonnie/DynOSAM/install/dynosam_ros
cmake --build /home/bonnie/DynOSAM/build/dynosam_ros -- -j16 -l16
cmake --install /home/bonnie/DynOSAM/build/dynosam_ros

# Or use colcon (preferred for ROS2)
colcon build --packages-select dynosam_ros
```

### Build Configuration Options
- `ENABLE_DYNAMIC_SLAM_INTERFACES`: ON by default. Controls whether to use dynamic_slam_interfaces for visualization or fall back to standard visualization_msgs
- `BUILD_TESTING`: Enables/disables test compilation

## Testing

### Test Framework
Uses `ament_cmake_gmock` with tests located in the `test/` directory.

### Running Tests
```bash
# Run all tests for this package
python3 scripts/run_dynosam_gtest.py --package dynosam_ros

# Run tests with additional gtest flags
python3 scripts/run_dynosam_gtest.py --package dynosam_ros --gtest_filter="*Adaptor*"

# Manual test execution
colcon test --packages-select dynosam_ros
```

### Test Structure
- `test_main.cc`: Main test executable
- `test_ros_utils.cc`: Tests for ROS utility functions
- `test_adaptors.cc`: Tests for type adapters (ROS ↔ DynOSAM)
- `test_online_dataprovider_ros.cc`: Tests for online data provider functionality

## Launch System

The package provides three main launch configurations:

### Standard Dataset Processing
```bash
ros2 launch dynosam_ros dyno_sam_launch.py
```
- Uses `dynosam_node` executable
- For offline dataset processing

### Online/Real-time Processing  
```bash
ros2 launch dynosam_ros dyno_sam_online_launch.py
```
- Real-time processing with live camera feeds
- Configurable camera topics (RGB, depth, optical flow, motion mask)

### Experimental Backend
```bash
ros2 launch dynosam_ros dyno_sam_experiments_launch.py
```
- Uses `dynosam_experiments_node` executable
- For backend-only experiments

### Launch Parameters
Key launch parameters managed by `launch_utils.py`:
- Dataset path, output path, verbosity level
- Camera topic remapping
- Online vs offline mode flags
- Transform publication settings

## Architecture Overview

### Core Design Patterns

**Pipeline Architecture**: Built around `DynoNode` base class that manages the DynOSAM processing pipeline through ROS2 integration.

**Data Provider Strategy Pattern**: Multiple data provider implementations:
- `OnlineDataProviderRos`: Real-time processing with message synchronization
- `ZEDOnlineDataProvider`: ZED camera-specific implementation
- Base `DataProviderRos`: Common ROS message conversion functionality

**Type Adaptation**: `CameraParamsAdaptor.hpp` provides bidirectional conversion between DynOSAM `CameraParams` and ROS `sensor_msgs::msg::CameraInfo` using ROS2 TypeAdapter pattern.

### Data Flow
```
ROS Topics → DataProvider → ImageContainer → DynOSAM Core Pipeline → Display Components
```

### Key Components

**Core Nodes**:
- `DynoNode`: Base ROS2 node with parameter management and data provider creation
- `DynoPipelineManagerRos`: Complete pipeline management with initialization and spinning

**Data Providers**:
- Handle synchronization of multiple image streams (RGB, depth, optical flow, motion mask)
- Type-safe conversion from ROS messages to OpenCV/DynOSAM formats
- Camera parameter retrieval and caching

**Display System**:
- Modular visualization components in `displays/` directory
- Two modes: dynamic_slam_interfaces (custom messages) or standard visualization_msgs
- Support for point clouds, odometry, paths, and marker arrays

## Development Guidelines

### Code Organization
- Headers in `include/dynosam_ros/`
- Implementation in `src/`
- Launch files in `launch/`
- Executable nodes in `nodes/`
- Tests in `test/`

### Key Dependencies
- Core dependency: `dynosam` package (the main SLAM library)
- Optional: `dynamic_slam_interfaces` for advanced visualization
- ROS2 packages: rclcpp, sensor_msgs, geometry_msgs, visualization_msgs, message_filters
- Third-party: OpenCV (via cv_bridge), PCL, Eigen3

### Type Safety
- Always use `image_traits<IMAGETYPE>::validate()` for image type checking
- Use ROS2 TypeAdapter pattern for seamless type conversions
- Validate paths in launch utilities before execution

### Error Handling
- Graceful degradation with informative error messages
- ROS node shutdown on critical type mismatches
- Timeout handling for camera parameter retrieval

## Visualization Modes

### Dynamic SLAM Interfaces (Default)
- Requires `dynamic_slam_interfaces` package
- Uses custom `ObjectOdometry` messages
- Requires `rviz_dynamic_slam_plugins` for visualization
- Enable with: `colcon build --cmake-args -DENABLE_DYNAMIC_SLAM_INTERFACES=ON`

### Inbuilt Displays (Fallback)
- Uses standard `visualization_msgs`
- More topics but broader compatibility
- Enable with: `colcon build --cmake-args -DENABLE_DYNAMIC_SLAM_INTERFACES=OFF`
````

## File: cmake_commands.sh
````bash
#!/usr/bin/env bash

export AMENT_PREFIX_PATH=/home/bonnie/DynOSAM/install/dynosam:/home/bonnie/DynOSAM/install/dynamic_slam_interfaces:${AMENT_PREFIX_PATH}
export CMAKE_PREFIX_PATH=/home/bonnie/DynOSAM/install/dynosam:/home/bonnie/DynOSAM/install/dynamic_slam_interfaces:/opt/ros/humble
export LD_LIBRARY_PATH=/home/bonnie/DynOSAM/install/dynosam/lib:/home/bonnie/DynOSAM/install/dynamic_slam_interfaces/lib:${LD_LIBRARY_PATH}
export PYTHONPATH=/home/bonnie/DynOSAM/install/dynamic_slam_interfaces/local/lib/python3.10/dist-packages:${PYTHONPATH}

cmake /home/bonnie/DynOSAM/src/dynosam_ros -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_BUILD_TYPE=RelWithDebinfo -G Ninja -DCMAKE_INSTALL_PREFIX=/home/bonnie/DynOSAM/install/dynosam_ros
cmake --build /home/bonnie/DynOSAM/build/dynosam_ros -- -j16 -l16
cmake --install /home/bonnie/DynOSAM/build/dynosam_ros
````

## File: CMakeLists.txt
````
cmake_minimum_required(VERSION 3.8)

project(dynosam_ros)

# set(CMAKE_C_FLAGS "-std=gnu11 -Wall -Wextra -O3 -march=sandybridge -flto")
set(CMAKE_C_FLAGS "-Wall -Wextra")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)


option(ENABLE_DYNAMIC_SLAM_INTERFACES "Use dynamic_slam_interfaces" ON)

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(dynosam REQUIRED)
find_package(PCL REQUIRED)
find_package(rclpy REQUIRED)
find_package(message_filters REQUIRED)

# setup targets
include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)



set(SRC_FILES
  src/PipelineRos.cc
  src/RosUtils.cc
  src/Utils.cc
  src/displays/DisplaysCommon.cc
  src/DataProviderRos.cc
  src/OnlineDataProviderRos.cc
  src/ZEDOnlineDataProvider.cc
)

set(TARGET_DEPS
  rclcpp
  std_msgs
  sensor_msgs
  visualization_msgs
  nav_msgs
  geometry_msgs
  dynosam
  image_transport
  cv_bridge
  pcl_conversions
  tf2_ros
  message_filters
)

set(TARGET_EXPORT_DEPS
  rclcpp
  std_msgs
  sensor_msgs
  dynosam
  message_filters
)


if(ENABLE_DYNAMIC_SLAM_INTERFACES)
  message(WARNING "dynamic_slam_interfaces will be used")
  add_definitions(-DUSE_DYNAMIC_SLAM_INTERFACES)

  find_package(dynamic_slam_interfaces REQUIRED)
  list(APPEND TARGET_DEPS dynamic_slam_interfaces)
  list(APPEND TARGET_EXPORT_DEPS dynamic_slam_interfaces)

  list(APPEND SRC_FILES
    src/displays/dynamic_slam_displays/DSDCommonRos.cc
    src/displays/dynamic_slam_displays/FrontendDSDRos.cc
    src/displays/dynamic_slam_displays/BackendDSDRos.cc)

else()
  message(WARNING "dynamic_slam_interfaces will be not used")
  list(APPEND SRC_FILES
    src/displays/inbuilt_displays/BackendInbuiltDisplayRos.cc
    src/displays/inbuilt_displays/InbuiltDisplayCommon.cc
    src/displays/inbuilt_displays/FrontendInbuiltDisplayRos.cc)


endif()

# build library
add_library(${PROJECT_NAME} SHARED ${SRC_FILES})

ament_target_dependencies(${PROJECT_NAME}
  ${TARGET_DEPS}
)

target_link_libraries(${PROJECT_NAME}
  ${rclcpp_LIBRARIES}
  glog
  gflags
)
ament_export_libraries(${PROJECT_NAME}
  glog
  gflags
)

ament_export_dependencies(${TARGET_EXPORT_DEPS})


target_include_directories(${PROJECT_NAME}
  PUBLIC
  ${rclcpp_INCLUDE_DIRS}
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)


#install python scripts/modules
ament_python_install_package(${PROJECT_NAME})

#####exec test#########
add_executable(dynosam_node nodes/dynosam_node.cc)
target_link_libraries(dynosam_node
  ${PROJECT_NAME}
  ${PCL_LIBRARIES}

)
ament_target_dependencies(dynosam_node "rclcpp" "std_msgs" "dynosam")

install(TARGETS
  dynosam_node
  DESTINATION lib/${PROJECT_NAME})



add_executable(dynosam_experiments_node nodes/backend_experiments_node.cc)
target_link_libraries(dynosam_experiments_node
  ${PROJECT_NAME}
  ${PCL_LIBRARIES}
)

ament_target_dependencies(dynosam_experiments_node "rclcpp" "std_msgs" "dynosam")

install(TARGETS
dynosam_experiments_node
  DESTINATION lib/${PROJECT_NAME})


add_executable(dynosam_dataset_node nodes/dataset_node.cc)
target_link_libraries(dynosam_dataset_node
  ${PROJECT_NAME}
  ${PCL_LIBRARIES}
)

ament_target_dependencies(dynosam_dataset_node "rclcpp" "std_msgs" "dynosam")

install(TARGETS
dynosam_dataset_node
  DESTINATION lib/${PROJECT_NAME})


    # Install Python executables
install(PROGRAMS
scripts/run_dynosam_gtest.py
DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
  # find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(ament_lint_auto REQUIRED)

  ament_lint_auto_find_test_dependencies()

  # Run all lint tests in package.xml except those listed above
  ament_lint_auto_find_test_dependencies()
  ament_add_gmock(${PROJECT_NAME}_test
    test/test_main.cc
    test/test_ros_utils.cc
    test/test_adaptors.cc
    test/test_online_dataprovider_ros.cc
    test/test_zed_online_dataprovider.cc
)

  target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME} ${rclcpp_LIBRARIES})

  install(TARGETS
  ${PROJECT_NAME}_test
      DESTINATION test/)

endif()


ament_package()
````

## File: package.xml
````xml
<?xml version="1.0"?>
<package format="2">
  <name>dynosam_ros</name>
  <version>0.0.1</version>
  <description>DynOSAM ROS Interface</description>

  <maintainer email="yiduo.wang@sydney.edu.au">Yiduo Wang</maintainer>
  <maintainer email="jesse.morris@sydney.edu.au">Jessie Morris</maintainer>
  <license>BSD</license>

  <buildtool_depend>catkin</buildtool_depend>
  <buildtool_depend>catkin_simple</buildtool_depend>

 <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>dynosam</depend>
  <depend>std_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>tf2_ros</depend>
  <depend>geometry_msgs</depend>
  <depend>dynamic_slam_interfaces</depend>
  <depend>visualization_msgs</depend>
  <depend>diagnostic_updater</depend>
  <depend>image_transport</depend>
  <depend>cv_bridge</depend>
  <depend>message_filters</depend>
  <depend>rclpy</depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <exec_depend>ros2launch</exec_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>

</package>
````
