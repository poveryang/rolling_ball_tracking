#include <gtest/gtest.h>
#include <opencv2/opencv.hpp>
#include <thread>
#include <chrono>
#include <filesystem>

#include "ball_tracker_interface.h"

class BallTrackingTest : public ::testing::Test {
protected:
    void SetUp() override {
        // 检查配置文件是否存在
        std::filesystem::path config_path = "config/balls_config.json";
        if (!std::filesystem::exists(config_path)) {
            FAIL() << "配置文件不存在: " << config_path;
        }

        // 检查视频文件是否存在
        std::filesystem::path video_path = "test/test_video.MOV";
        if (!std::filesystem::exists(video_path)) {
            FAIL() << "测试视频文件不存在: " << video_path;
        }
    }

    void TearDown() override {
        if (interface_) {
            interface_->StopTracking();
            interface_.reset();
        }
    }

    std::unique_ptr<BallTrackerInterface> interface_;
    std::vector<BallStatus> last_status_;
    bool callback_called_ = false;

public:
    // 回调函数改为公共成员
    void OnBallStatusUpdate(const std::vector<BallStatus>& status) {
        last_status_ = status;
        callback_called_ = true;
        std::cout << "收到小球状态更新：" << std::endl;
        for (const auto& ball : status) {
            std::cout << "ID: " << ball.id 
                      << ", 位置: (" << ball.x << ", " << ball.y << ")"
                      << ", 速度: (" << ball.vx << ", " << ball.vy << ")"
                      << std::endl;
        }
    }
};

// 测试初始化跟踪（读取视频文件）
TEST_F(BallTrackingTest, TestInitTrack) {
    // 初始化跟踪器
    interface_ = std::make_unique<BallTrackerInterface>("config/balls_config.json", 
                                                      std::make_pair(356, 782));

    // 初始化相机（读取视频文件）
    std::cout << "初始化相机..." << std::endl;
    ASSERT_TRUE(interface_->InitializeCamera("test/test_video.MOV")) 
        << "无法初始化相机";

    std::cout << "开始初始化轨迹..." << std::endl;
    int result = interface_->InitTrack("config/trajectory.json");
    std::cout << "初始化轨迹结果: " << result << std::endl;
    EXPECT_EQ(result, 0);
}

// 测试小球跟踪（读取视频文件）
TEST_F(BallTrackingTest, TestVideoTracking) {
    // 初始化跟踪器
    interface_ = std::make_unique<BallTrackerInterface>("config/balls_config.json", 
                                                      std::make_pair(356, 782));

    // 初始化相机（读取视频文件）
    std::cout << "初始化相机..." << std::endl;
    ASSERT_TRUE(interface_->InitializeCamera("test/test_video.MOV")) 
        << "无法初始化相机";

    // 注册回调
    interface_->RegisterBallStatusCallback(
        std::bind(&BallTrackingTest::OnBallStatusUpdate, this, std::placeholders::_1));

    std::cout << "开始视频跟踪..." << std::endl;
    interface_->StartTracking();

    // 等待一段时间，让跟踪器处理一些帧
    std::cout << "等待5秒..." << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(5));

    // 检查回调是否被调用
    EXPECT_TRUE(callback_called_);
    EXPECT_FALSE(last_status_.empty());

    std::cout << "停止跟踪..." << std::endl;
    interface_->StopTracking();
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
} 