#include <gtest/gtest.h>
#include <opencv2/opencv.hpp>
#include <thread>
#include <chrono>
#include <filesystem>

#include "ball_tracker_interface.h"

class BallTrackingTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Check if config file exists
        std::filesystem::path config_path = "config/balls_config.json";
        if (!std::filesystem::exists(config_path)) {
            FAIL() << "Config file not found: " << config_path;
        }

        // Check if video file exists
        std::filesystem::path video_path = "test/test_video.MOV";
        if (!std::filesystem::exists(video_path)) {
            FAIL() << "Test video file not found: " << video_path;
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
    // Make callback function public
    void OnBallStatusUpdate(const std::vector<BallStatus>& status) {
        last_status_ = status;
        callback_called_ = true;
        std::cout << "Received ball status update:" << std::endl;
        for (const auto& ball : status) {
            std::cout << "ID: " << ball.id 
                      << ", Position: (" << ball.x << ", " << ball.y << ")"
                      << ", Velocity: (" << ball.vx << ", " << ball.vy << ")"
                      << std::endl;
        }
    }
};

// Test initialization tracking (read video file)
TEST_F(BallTrackingTest, TestInitTrack) {
    // Initialize tracker
    interface_ = std::make_unique<BallTrackerInterface>("config/balls_config.json", 
                                                      std::make_pair(356, 782));

    // Initialize camera (read video file)
    std::cout << "Initializing camera..." << std::endl;
    ASSERT_TRUE(interface_->InitializeCamera("test/test_video.MOV")) 
        << "Failed to initialize camera";

    std::cout << "Starting trajectory initialization..." << std::endl;
    int result = interface_->InitTrack("config/trajectory.json");
    std::cout << "Trajectory initialization result: " << result << std::endl;
    EXPECT_EQ(result, 0);
}

// Test ball tracking (read video file)
TEST_F(BallTrackingTest, TestVideoTracking) {
    // Initialize tracker
    interface_ = std::make_unique<BallTrackerInterface>("config/balls_config.json", 
                                                      std::make_pair(356, 782));

    // Initialize camera (read video file)
    std::cout << "Initializing camera..." << std::endl;
    ASSERT_TRUE(interface_->InitializeCamera("test/test_video.MOV")) 
        << "Failed to initialize camera";

    // Register callback
    interface_->RegisterBallStatusCallback(
        std::bind(&BallTrackingTest::OnBallStatusUpdate, this, std::placeholders::_1));

    std::cout << "Starting video tracking..." << std::endl;
    interface_->StartTracking();

    // Wait for some time to let tracker process some frames
    std::cout << "Waiting for 5 seconds..." << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(5));

    // Check if callback was called
    EXPECT_TRUE(callback_called_);
    EXPECT_FALSE(last_status_.empty());

    std::cout << "Stopping tracking..." << std::endl;
    interface_->StopTracking();
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
} 
