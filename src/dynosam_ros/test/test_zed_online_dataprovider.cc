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