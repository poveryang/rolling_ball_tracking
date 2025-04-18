#include <gtest/gtest.h>
#include <opencv2/opencv.hpp>
#include "camera_control.h"
#include <filesystem>
#include <thread>

class CameraControlTest : public ::testing::Test {
protected:
    void SetUp() override {}
    void TearDown() override {}
    BallTrackerCamera camera_;
};

// Test basic USB camera functionality
TEST_F(CameraControlTest, TestUSBCamera) {
    // Try to open USB camera
    if (camera_.Open("0", -1, -1, -1, CameraSourceType::USB_CAMERA)) {
        std::cout << camera_.GetInfo() << std::endl;

        // Test image capture
        cv::Mat frame;
        bool capture_success = camera_.Capture(frame);
        EXPECT_TRUE(capture_success);
        if (capture_success) {
            EXPECT_FALSE(frame.empty());
            std::cout << "Successfully captured image, resolution: " 
                      << frame.cols << "x" << frame.rows << std::endl;
        }

        camera_.Close();
    } else {
        std::cout << "Failed to open USB camera, please check connection" << std::endl;
        GTEST_SKIP();
    }
}

// Test video file reading
TEST_F(CameraControlTest, TestVideoFile) {
    const std::string test_video = "test/test_video.mp4";
    if (camera_.Open(test_video, -1, -1, -1, CameraSourceType::VIDEO_FILE)) {
        std::cout << camera_.GetInfo() << std::endl;

        // Test image reading
        cv::Mat frame;
        bool read_success = camera_.Capture(frame);
        EXPECT_TRUE(read_success);
        if (read_success) {
            EXPECT_FALSE(frame.empty());
            std::cout << "Successfully read video frame, resolution: " 
                      << frame.cols << "x" << frame.rows << std::endl;
        }
        camera_.Close();
    } else {
        std::cout << "Failed to open video file: " << test_video << std::endl;
        GTEST_SKIP();
    }
}

// Test Huarui camera (currently reserved interface)
TEST_F(CameraControlTest, TestHuaruiCamera) {
    if (camera_.Open("SN123456", 640, 480, 30, CameraSourceType::HUARUI_CAMERA)) {
        std::cout << "Huarui camera information:" << std::endl;
        std::cout << camera_.GetInfo() << std::endl;

        // Test image capture
        cv::Mat frame;
        bool capture_success = camera_.Capture(frame);
        EXPECT_TRUE(capture_success);
        if (capture_success) {
            EXPECT_FALSE(frame.empty());
            std::cout << "Successfully captured image, resolution: " 
                      << frame.cols << "x" << frame.rows << std::endl;
        }
        camera_.Close();
    } else {
        std::cout << "Huarui camera interface not implemented yet" << std::endl;
        GTEST_SKIP();
    }
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
