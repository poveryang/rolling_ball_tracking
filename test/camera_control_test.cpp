#include <gtest/gtest.h>
#include <opencv2/opencv.hpp>
#include "camera_control.h"

class CameraControlTest : public ::testing::Test {
protected:
    void SetUp() override {}
    void TearDown() override {}
    BallTrackerCamera camera_;
};

// 测试USB相机基本功能
TEST_F(CameraControlTest, TestUSBCamera) {
    // 尝试打开USB相机
    if (camera_.Open("0", -1, -1, -1, CameraSourceType::USB_CAMERA)) {
        std::cout << camera_.GetInfo() << std::endl;

        // 测试图像采集
        cv::Mat frame;
        bool capture_success = camera_.Capture(frame);
        EXPECT_TRUE(capture_success);
        if (capture_success) {
            EXPECT_FALSE(frame.empty());
            std::cout << "成功采集图像, 分辨率: " 
                      << frame.cols << "x" << frame.rows << std::endl;
        }

        camera_.Close();
    } else {
        std::cout << "未能打开USB相机, 请检查连接" << std::endl;
        GTEST_SKIP();
    }
}

// 测试视频文件读取
TEST_F(CameraControlTest, TestVideoFile) {
    const std::string test_video = "test/test_video.mp4";
    if (camera_.Open(test_video, -1, -1, -1, CameraSourceType::VIDEO_FILE)) {
        std::cout << camera_.GetInfo() << std::endl;

        // 测试图像读取
        cv::Mat frame;
        bool read_success = camera_.Capture(frame);
        EXPECT_TRUE(read_success);
        if (read_success) {
            EXPECT_FALSE(frame.empty());
            std::cout << "成功读取视频帧, 分辨率: " 
                      << frame.cols << "x" << frame.rows << std::endl;
        }
        camera_.Close();
    } else {
        std::cout << "未能打开视频文件: " << test_video << std::endl;
        GTEST_SKIP();
    }
}

// 测试华睿相机（目前为预留接口）
TEST_F(CameraControlTest, TestHuaruiCamera) {
    if (camera_.Open("SN123456", 640, 480, 30, CameraSourceType::HUARUI_CAMERA)) {
        std::cout << "华睿相机信息：" << std::endl;
        std::cout << camera_.GetInfo() << std::endl;

        // 测试图像采集
        cv::Mat frame;
        bool capture_success = camera_.Capture(frame);
        EXPECT_TRUE(capture_success);
        if (capture_success) {
            EXPECT_FALSE(frame.empty());
            std::cout << "成功采集图像, 分辨率: " 
                      << frame.cols << "x" << frame.rows << std::endl;
        }
        camera_.Close();
    } else {
        std::cout << "华睿相机接口尚未实现" << std::endl;
        GTEST_SKIP();
    }
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}