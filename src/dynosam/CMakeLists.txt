cmake_minimum_required(VERSION 3.8)

project(dynosam)

# set(CMAKE_C_FLAGS "-std=gnu11 -Wall -Wextra -O3 -march=sandybridge -flto")
set(CMAKE_C_FLAGS "-Wall -Wextra")

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

set(CMAKE_CXX_STANDARD 17)


list(INSERT CMAKE_MODULE_PATH 0 ${CMAKE_SOURCE_DIR}/cmake)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(CUDA REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(opengv REQUIRED)
find_package(GTSAM REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(nlohmann_json 3.2.0 REQUIRED) #nlohmann-json3-dev
find_package(TBB REQUIRED) # should be packaged with GTSAM
find_package(config_utilities REQUIRED)
find_package(ZED REQUIRED)


find_package(Boost REQUIRED)
if (NOT TARGET Boost::boost)
    add_library(Boost::boost INTERFACE IMPORTED)
    set_target_properties(Boost::boost PROPERTIES
            INTERFACE_LINK_LIBRARIES "${Boost_LIBRARIES}"
            INTERFACE_INCLUDE_DIRECTORIES "${Boost_INCLUDE_DIRS}")
endif ()


# setup targets
include_directories(
        include
        ${rclcpp_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${GTSAM_INCLUDE_DIR}
        ${PCL_INCLUDE_DIRS}
        ${ZED_INCLUDE_DIRS}
        ${CUDA_INCLUDE_DIRS}
)

# build library
add_library(${PROJECT_NAME} SHARED

        src/common/Types.cc
        src/common/CameraParams.cc
        src/common/Camera.cc
        src/common/GroundTruthPacket.cc
        src/common/ImageContainer.cc
        src/common/ImageTypes.cc
        src/common/StructuredContainers.cc
        src/common/DynamicObjects.cc
        src/common/Map.cc
        src/common/MapNodes.cc
        src/common/PointCloudProcess.cc
        src/common/Algorithms.cc
        src/common/StereoCamera.cc
        src/common/ZEDCamera.cc

        src/common/byte_tracker/ByteTracker.cc
        src/common/byte_tracker/Detection.cc
        src/common/byte_tracker/KalmanFilter.cc
        src/common/byte_tracker/Lapjv.cc
        src/common/byte_tracker/Rect.cc
        src/common/byte_tracker/Track.cc

        src/dataprovider/DataInterfacePipeline.cc
        src/dataprovider/DataProvider.cc
        src/dataprovider/DatasetLoader.cc
        src/dataprovider/DataProviderUtils.cc
        src/dataprovider/DataProviderFactory.cc
        src/dataprovider/VirtualKittidataProvider.cc
        src/dataprovider/ClusterSlamDataProvider.cc
        src/dataprovider/OMDDataProvider.cc
        src/dataprovider/DatasetProvider.cc
        src/dataprovider/ProjectAriaDataProvider.cc
        src/dataprovider/ZEDDataProvider.cc

        src/frontend/imu/ThreadSafeImuBuffer.cc
        src/frontend/vision/Feature.cc
        src/frontend/vision/UndistortRectifier.cc
        src/frontend/vision/Frame.cc
        src/frontend/FrontendPipeline.cc
        src/frontend/FrontendModule.cc
        src/frontend/FrontendParams.cc
        src/frontend/RGBDInstanceFrontendModule.cc
        src/frontend/vision/ORBextractor.cc
        src/frontend/vision/FeatureTracker.cc
        src/frontend/vision/ObjectTracker.cc
        src/frontend/vision/VisionTools.cc
        src/frontend/vision/MotionSolver.cc
        src/frontend/vision/StereoMatcher.cc
        src/frontend/vision/StaticFeatureTracker.cc
        src/frontend/vision/TrackerParams.cc
        src/frontend/vision/FeatureTrackerBase.cc
        src/frontend/vision/FeatureDetector.cc
        src/frontend/RGBDInstance-Definitions.cc
        src/frontend/Frontend-Definitions.cc
        src/frontend/anms/NonMaximumSupression.cc
        src/frontend/anms/anms.cc

        src/backend/BackendDefinitions.cc
        src/backend/BackendPipeline.cc
        src/backend/BackendModule.cc
        src/backend/BackendParams.cc
        src/backend/FactorGraphTools.cc
        src/backend/DynamicPointSymbol.cc
        src/backend/RGBDBackendModule.cc
        src/backend/rgbd/WorldMotionEstimator.cc
        src/backend/rgbd/WorldPoseEstimator.cc

        src/factors/LandmarkMotionTernaryFactor.cc
        src/factors/Pose3FlowProjectionFactor.cc
        src/factors/ObjectKinematicFactor.cc
        src/factors/LandmarkMotionPoseFactor.cc
        src/factors/LandmarkPoseSmoothingFactor.cc

        src/pipeline/PipelineManager.cc
        src/pipeline/PipelineParams.cc
        src/pipeline/PipelineBase.cc
        src/pipeline/PipelineSpinner.cc

        src/logger/Logger.cc

        src/utils/OpenCVUtils.cc
        src/utils/GtsamUtils.cc
        src/utils/Spinner.cc
        src/utils/Statistics.cc
        src/utils/TimingStats.cc
        src/utils/Metrics.cc
        src/utils/CsvParser.cc
        src/utils/Histogram.cc
        src/utils/JsonUtils.cc

        src/visualizer/ColourMap.cc
        src/visualizer/VisualizerPipelines.cc
        src/visualizer/OpenCVFrontendDisplay.cc
        src/visualizer/Visualizer-Definitions.cc
)

# target_compile_definitions(${PROJECT_NAME}
#   PRIVATE "MINIMAL_COMPOSITION_DLL")

# eigen3_cmake_module
# Eigen3

ament_target_dependencies(${PROJECT_NAME}
        rclcpp
        std_msgs
        sensor_msgs
        opengv
        GTSAM
        OpenCV
        ZED
        CUDA
)

target_link_libraries(${PROJECT_NAME}
        # PRIVATE
        opengv
        nlohmann_json::nlohmann_json
        config_utilities::config_utilities
        # PUBLIC
        ${rclcpp_LIBRARIES}
        glog
        gflags
        gtsam
        gtsam_unstable
        # gtsamDebug
        # gtsam_unstableDebug
        Boost::boost
        png # We need png++ (libpng-dev) for png manipulation in VirtualKittiDatasetLoader
        # Eigen3::Eigen
        ${OpenCV_LIBRARIES}
        ${PCL_LIBRARIES}
        TBB::tbb
        ${ZED_LIBRARIES}
        # PRIVATE
)


ament_export_libraries(${PROJECT_NAME}
        glog
        gflags
        opengv
        Boost::boost
        config_utilities::config_utilities
        gtsam
        # gtsamDebug
        ${OpenCV_LIBRARIES}
        png
        TBB::tbb
)


set(DYNO_EXPORT_DEPS
        rclcpp
        std_msgs
        sensor_msgs
        OpenCV opengv
        GTSAM
        eigen3_cmake_module
        Eigen3
        ZED
        CUDA
)


ament_export_dependencies(${DYNO_EXPORT_DEPS})


target_include_directories(${PROJECT_NAME}
        PUBLIC
        ${OpenCV_INCLUDE_DIRS}
        ${GFLAGS_INCLUDE_DIRS}
        ${GLOG_INCLUDE_DIRS}
        ${GTSAM_INCLUDE_DIR}
        ${OPENGV_INCLUDE_DIR}
        ${ZED_INCLUDE_DIRS}
        ${CUDA_INCLUDE_DIRS}
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

ament_export_include_directories(include ${GTSAM_INCLUDE_DIR} ${OPENGV_INCLUDE_DIR} ${ZED_INCLUDE_DIRS} ${CUDA_INCLUDE_DIRS}) # --- ZED SDK Integration: Export ZED include directory ---
ament_export_libraries(${PROJECT_NAME})

install(
        DIRECTORY include/
        DESTINATION include
)

# install test include files for usage in other packges
# header only files will work since the .cc files are not included in the exported library
install(
        DIRECTORY test/internal/
        DESTINATION include/dynosam/test
        FILES_MATCHING PATTERN "*.hpp"
)

install(
        TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

## This is VERY IMPORTANT so that dynosam_ros can find the params folder using ament indexing!!
install(DIRECTORY
        params
        DESTINATION share/${PROJECT_NAME}
)


#####exec test#########
add_executable(dyno_sam example/dyno_sam.cc)
target_link_libraries(dyno_sam
        ${PROJECT_NAME}
)
ament_target_dependencies(dyno_sam rclcpp std_msgs)


install(TARGETS
        dyno_sam
        DESTINATION lib/${PROJECT_NAME})


if (BUILD_TESTING)
    find_package(ament_cmake_gmock REQUIRED)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()

    # Define common sources needed for most/all test executables
    # test_main.cc is the GTest entry point.
    # internal/simulator.cc and internal/tmp_file.cc are helper implementations.
    set(COMMON_TEST_COMPILE_SOURCES
            test/test_main.cc
            test/internal/simulator.cc
            test/internal/tmp_file.cc
    )

    include_directories(test)

    # --- Group 1: Core Utilities ---
    set(CORE_UTILS_TEST_FILES
            test/core_utils/test_types.cc
            test/core_utils/test_csv.cc
            test/core_utils/test_numerical.cc
            test/core_utils/test_histogram.cc
            test/core_utils/test_structured_containers.cc
            test/core_utils/test_code_concepts.cc
    )
    ament_add_gmock(dynosam_test_core_utils ${COMMON_TEST_COMPILE_SOURCES} ${CORE_UTILS_TEST_FILES})
    target_link_libraries(dynosam_test_core_utils ${PROJECT_NAME} ${rclcpp_LIBRARIES})

    # --- Group 2: Camera Functionality ---
    set(CAMERA_TEST_FILES
            test/camera/test_camera_params.cc
            test/camera/test_camera.cc
    )
    ament_add_gmock(dynosam_test_camera ${COMMON_TEST_COMPILE_SOURCES} ${CAMERA_TEST_FILES})
    target_link_libraries(dynosam_test_camera ${PROJECT_NAME} ${rclcpp_LIBRARIES})

    # --- Group 3: Thread Safety ---
    set(THREAD_SAFETY_TEST_FILES
            test/thread_safety/thread_safe_queue_tests.cc
            test/thread_safety/thread_safe_temporal_buffer_test.cc
            test/thread_safety/test_threadsafe_imu_buffer.cc
    )
    ament_add_gmock(dynosam_test_thread_safety ${COMMON_TEST_COMPILE_SOURCES} ${THREAD_SAFETY_TEST_FILES})
    target_link_libraries(dynosam_test_thread_safety ${PROJECT_NAME} ${rclcpp_LIBRARIES})

    # --- Group 4: Data Providers ---
    set(DATAPROVIDERS_TEST_FILES
            test/dataproviders/test_dataset_provider.cc
    )
    ament_add_gmock(dynosam_test_dataproviders ${COMMON_TEST_COMPILE_SOURCES} ${DATAPROVIDERS_TEST_FILES})
    target_link_libraries(dynosam_test_dataproviders ${PROJECT_NAME} ${rclcpp_LIBRARIES})

    # --- Group 5: Backend ---
    set(BACKEND_TEST_FILES
            test/backend/test_backend_structures.cc
            test/backend/test_dynamic_point_symbol.cc
            test/backend/test_factors.cc
            test/backend/test_rgbd_backend.cc
            # test/backend/test_landmark_quadric_factor.cc
            # test/backend/test_triangulation.cc
    )
    ament_add_gmock(dynosam_test_backend ${COMMON_TEST_COMPILE_SOURCES} ${BACKEND_TEST_FILES})
    target_link_libraries(dynosam_test_backend ${PROJECT_NAME} ${rclcpp_LIBRARIES})

    # --- Group 6: Algorithms ---
    set(ALGORITHMS_TEST_FILES
            test/algorithms/test_algorithms.cc
    )
    ament_add_gmock(dynosam_test_algorithms ${COMMON_TEST_COMPILE_SOURCES} ${ALGORITHMS_TEST_FILES})
    target_link_libraries(dynosam_test_algorithms ${PROJECT_NAME} ${rclcpp_LIBRARIES})

    # --- Group 7: Vision & Map ---
    set(VISION_MAP_TEST_FILES
            test/vision_map/test_tools.cc
            test/vision_map/test_map.cc
    )
    ament_add_gmock(dynosam_test_vision_map ${COMMON_TEST_COMPILE_SOURCES} ${VISION_MAP_TEST_FILES})
    target_link_libraries(dynosam_test_vision_map ${PROJECT_NAME} ${rclcpp_LIBRARIES})

    # --- Group 8: Pipelines & Params ---
    set(PIPELINES_PARAMS_TEST_FILES
            test/pipelines_params/test_pipelines.cc
            test/pipelines_params/test_params.cc
    )
    ament_add_gmock(dynosam_test_pipelines_params ${COMMON_TEST_COMPILE_SOURCES} ${PIPELINES_PARAMS_TEST_FILES})
    target_link_libraries(dynosam_test_pipelines_params ${PROJECT_NAME} ${rclcpp_LIBRARIES})

    # --- Group 9: Visualization ---
    set(VISUALIZATION_TEST_FILES
            test/visualization/test_viz.cc
    )
    ament_add_gmock(dynosam_test_visualization ${COMMON_TEST_COMPILE_SOURCES} ${VISUALIZATION_TEST_FILES})
    target_link_libraries(dynosam_test_visualization ${PROJECT_NAME} ${rclcpp_LIBRARIES})

    # --- Group 10: ZED Camera ---
    set(ZED_CAMERA_TEST_FILES
            test/camera/ZEDCamera/test_ZED_camera.cc
    )
    ament_add_gmock(dynosam_test_zed_camera  ${ZED_CAMERA_TEST_FILES})
    target_link_libraries(dynosam_test_zed_camera ${PROJECT_NAME} ${rclcpp_LIBRARIES} ${ZED_LIBRARIES})

    # Install all new test executables
    install(TARGETS
            dynosam_test_core_utils
            dynosam_test_camera
            dynosam_test_thread_safety
            dynosam_test_dataproviders
            dynosam_test_backend
            dynosam_test_algorithms
            dynosam_test_vision_map
            dynosam_test_pipelines_params
            dynosam_test_visualization
            dynosam_test_zed_camera
            DESTINATION test/)

    #install data so it can be found on the share package path
    install(
            DIRECTORY test/data/
            DESTINATION test/data/
    )

endif ()

ament_package()
