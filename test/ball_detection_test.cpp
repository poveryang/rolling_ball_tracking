#include <gtest/gtest.h>
#include <opencv2/opencv.hpp>
#include "camera_control.h"
#include "ball_tracker_algo.h"
#include <filesystem>

class BallDetectionTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Ensure test data directory exists
        std::filesystem::path test_data_dir = "test_data";
        if (!std::filesystem::exists(test_data_dir)) {
            std::filesystem::create_directories(test_data_dir);
        }

        // Check if video file exists
        std::filesystem::path video_path = "test/test_video.MOV";
        if (!std::filesystem::exists(video_path)) {
            FAIL() << "Test video file not found: " << video_path;
        }
    }

    void TearDown() override {
        if (camera_.IsOpen()) {
            camera_.Close();
        }
    }

    // Calculate HSV mean and standard deviation for ROI
    void CalculateHSVParams(const cv::Mat& frame, const cv::Rect& roi, 
                          cv::Scalar& hsv_mean, cv::Scalar& hsv_stddev) {
        cv::Mat roi_frame = frame(roi);
        cv::Mat hsv;
        cv::cvtColor(roi_frame, hsv, cv::COLOR_BGR2HSV);
        
        // Calculate mean and standard deviation
        cv::Mat mean, stddev;
        cv::meanStdDev(hsv, mean, stddev);
        
        hsv_mean = cv::Scalar(mean.at<double>(0), mean.at<double>(1), mean.at<double>(2));
        hsv_stddev = cv::Scalar(stddev.at<double>(0), stddev.at<double>(1), stddev.at<double>(2));
    }

    BallTrackerCamera camera_;
    std::unique_ptr<BallTracker> ball_tracker_;
    cv::Rect selected_roi_;
    bool roi_selected_ = false;

    // Mouse callback function
    static void OnMouse(int event, int x, int y, int flags, void* userdata) {
        auto* test = static_cast<BallDetectionTest*>(userdata);
        static bool is_drawing = false;
        static cv::Point start_point;

        if (event == cv::EVENT_LBUTTONDOWN) {
            is_drawing = true;
            start_point = cv::Point(x, y);
        }
        else if (event == cv::EVENT_MOUSEMOVE && is_drawing) {
            test->selected_roi_ = cv::Rect(start_point, cv::Point(x, y));
        }
        else if (event == cv::EVENT_LBUTTONUP) {
            is_drawing = false;
            test->selected_roi_ = cv::Rect(start_point, cv::Point(x, y));
            test->roi_selected_ = true;
        }
    }
};

// Test video file detection
TEST_F(BallDetectionTest, TestVideoFileDetection) {
    // Initialize camera (file mode)
    ASSERT_TRUE(camera_.Open("test/test_video.MOV", -1, -1, -1, CameraSourceType::VIDEO_FILE))
        << "Failed to open test video file";

    // Use fixed HSV parameters and initial position
    cv::Scalar hsv_mean(37.30, 181.83, 252.62);
    cv::Scalar hsv_stddev(0.57, 19.56, 1.84);
    cv::Point2d init_pos(356, 782);

    // Create ball detector
    ball_tracker_ = std::make_unique<BallTracker>(1, "test_ball", hsv_mean, hsv_stddev, init_pos);
    
    std::cout << "HSV Parameters:" << std::endl;
    std::cout << "  Mean: " << hsv_mean << std::endl;
    std::cout << "  StdDev: " << hsv_stddev << std::endl;
    std::cout << "  Initial Position: (" << init_pos.x << ", " << init_pos.y << ")" << std::endl;

    // Main loop
    cv::Mat frame;
    while (camera_.Capture(frame)) {
        // Update ball detection
        ball_tracker_->UpdateWithImage(frame);
        auto status = ball_tracker_->GetStatus();
        
        // Get current ROI
        auto roi = ball_tracker_->GetROI();
        printf("roi: %d, %d, %d, %d\n", roi.x, roi.y, roi.width, roi.height);
        
        if (status.detected) {
            std::cout << "Ball detected:" << std::endl;
            std::cout << "  ID: " << status.id << std::endl;
            std::cout << "  Position: (" << status.x << ", " << status.y << ")" << std::endl;
            std::cout << "  Velocity: (" << status.vx << ", " << status.vy << ")" << std::endl;
        }
    }
}

// Test interactive ROI selection from camera
TEST_F(BallDetectionTest, TestCameraInteractiveDetection) {
    // Initialize camera
    ASSERT_TRUE(camera_.Open("1", -1, -1, -1, CameraSourceType::USB_CAMERA))
        << "Failed to open USB camera";

    cv::Mat frame;
    const std::string window_name = "Ball Detection Preview";
    cv::namedWindow(window_name, cv::WINDOW_NORMAL);
    
    // Set mouse callback
    cv::setMouseCallback(window_name, OnMouse, this);

    // Main loop
    while (true) {
        // Capture image
        if (!camera_.Capture(frame)) {
            std::cout << "Image capture failed" << std::endl;
            continue;
        }

        cv::Mat display_frame = frame.clone();
        
        // If ROI is selected, calculate HSV parameters and initialize detector
        if (!ball_tracker_ && roi_selected_) {
            cv::Scalar hsv_mean, hsv_stddev;
            CalculateHSVParams(frame, selected_roi_, hsv_mean, hsv_stddev);
            
            // Use ROI center as initial position
            cv::Point2d init_pos(selected_roi_.x + selected_roi_.width/2.0,
                               selected_roi_.y + selected_roi_.height/2.0);
            ball_tracker_ = std::make_unique<BallTracker>(1, "test_ball", hsv_mean, hsv_stddev, init_pos);
            
            std::cout << "HSV Parameters calculated:" << std::endl;
            std::cout << "  Mean: " << hsv_mean << std::endl;
            std::cout << "  StdDev: " << hsv_stddev << std::endl;
            std::cout << "  Initial Position: (" << init_pos.x << ", " << init_pos.y << ")" << std::endl;
        }

        // Draw ROI selection box
        if (selected_roi_.width > 0 && selected_roi_.height > 0) {
            cv::rectangle(display_frame, selected_roi_, cv::Scalar(0, 255, 0), 2);
        }

        // If detector is initialized, perform ball detection
        if (ball_tracker_) {
            ball_tracker_->UpdateWithImage(frame);
            auto status = ball_tracker_->GetStatus();
            
            // Get current ROI
            auto roi = ball_tracker_->GetROI();
            printf("roi: %d, %d, %d, %d\n", roi.x, roi.y, roi.width, roi.height);
            
            // Draw ROI
            cv::rectangle(display_frame, roi, cv::Scalar(255, 0, 0), 2);  // Blue rectangle for ROI
            
            if (status.detected) {
                // Draw detected ball on image
                cv::circle(display_frame, cv::Point(static_cast<int>(status.x), static_cast<int>(status.y)), 10, cv::Scalar(0, 255, 0), 2);
                
                // Display ball status
                std::stringstream ss;
                ss << "ID: " << status.id 
                   << " Pos: (" << static_cast<int>(status.x) << ", " << static_cast<int>(status.y) << ")"
                   << " Vel: (" << static_cast<int>(status.vx) << ", " << static_cast<int>(status.vy) << ")";
                cv::putText(display_frame, ss.str(),
                           cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX,
                           0.7, cv::Scalar(0, 255, 0), 2);
            }
        }

        // Display instructions
        if (!roi_selected_) {
            cv::putText(display_frame, "Select ROI",
                       cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX,
                       0.7, cv::Scalar(0, 0, 255), 2);
        }

        cv::imshow(window_name, display_frame);
        
        // Press ESC to exit
        if (cv::waitKey(30) == 27) {
            break;
        }
    }

    cv::destroyAllWindows();
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
} 
