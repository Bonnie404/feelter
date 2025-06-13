from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Define ZED-specific arguments with appropriate defaults
    declared_arguments = [

        # ZED camera topics
        DeclareLaunchArgument('zed_rgb_topic', default_value='/zed/zed_node/rgb/image_rect_color', description='ZED RGB image topic'),
        DeclareLaunchArgument('zed_depth_topic', default_value='/zed/zed_node/depth/depth_registered', description='ZED depth image topic'),
        DeclareLaunchArgument('zed_camera_info_topic', default_value='/zed/zed_node/rgb/camera_info', description='ZED camera info topic'),
        DeclareLaunchArgument('zed_imu_topic', default_value='/zed/zed_node/imu/data', description='ZED IMU topic'),
        DeclareLaunchArgument('zed_semantic_mask_topic', default_value='/zed/zed_node/obj_det/semantic_mask', description='ZED Semantic Mask topic'),
        DeclareLaunchArgument('zed_detected_objects_topic', default_value='/zed/zed_node/obj_det/objects', description='ZED Detected Objects topic'),
        DeclareLaunchArgument('zed_optical_flow_topic', default_value='/zed/zed_node/left/optical_flow', description='ZED Optical Flow topic'),
        DeclareLaunchArgument('zed_skeletons_topic', default_value='/zed/zed_node/body_trk/skeletons', description='ZED Skeletons topic'),

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
            'zed.semantic_mask_topic': LaunchConfiguration('zed_semantic_mask_topic'),
            'zed.detected_objects_topic': LaunchConfiguration('zed_detected_objects_topic'),
            'zed.optical_flow_topic': LaunchConfiguration('zed_optical_flow_topic'),
            'zed.skeletons_topic': LaunchConfiguration('zed_skeletons_topic'),
            
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