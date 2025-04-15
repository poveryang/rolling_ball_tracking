#include <gtest/gtest.h>
#include <opencv2/opencv.hpp>
#include "camera_control.h"
#include "ball_tracker_algo.h"
#include <filesystem>

class BallDetectionTest : public ::testing::Test {
protected:
    void SetUp() override {
        // 确保测试数据目录存在
        std::filesystem::path test_data_dir = "test_data";
        if (!std::filesystem::exists(test_data_dir)) {
            std::filesystem::create_directories(test_data_dir);
        }

        // 检查视频文件是否存在
        std::filesystem::path video_path = "test/test_video.MOV";
        if (!std::filesystem::exists(video_path)) {
            FAIL() << "测试视频文件不存在: " << video_path;
        }
    }

    void TearDown() override {
        if (camera_.IsOpen()) {
            camera_.Close();
        }
    }

    // 计算ROI区域的HSV均值和标准差
    void CalculateHSVParams(const cv::Mat& frame, const cv::Rect& roi, 
                          cv::Scalar& hsv_mean, cv::Scalar& hsv_stddev) {
        cv::Mat roi_frame = frame(roi);
        cv::Mat hsv;
        cv::cvtColor(roi_frame, hsv, cv::COLOR_BGR2HSV);
        
        // 计算均值和标准差
        cv::Mat mean, stddev;
        cv::meanStdDev(hsv, mean, stddev);
        
        hsv_mean = cv::Scalar(mean.at<double>(0), mean.at<double>(1), mean.at<double>(2));
        hsv_stddev = cv::Scalar(stddev.at<double>(0), stddev.at<double>(1), stddev.at<double>(2));
    }

    BallTrackerCamera camera_;
    std::unique_ptr<BallTracker> ball_tracker_;
    cv::Rect selected_roi_;
    bool roi_selected_ = false;

    // 鼠标回调函数
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

// 测试从文件读取视频并检测小球
TEST_F(BallDetectionTest, TestVideoFileDetection) {
    // 初始化相机（读文件方式）
    ASSERT_TRUE(camera_.Open("test/test_video.MOV", -1, -1, -1, CameraSourceType::VIDEO_FILE))
        << "无法打开测试视频文件";

    // 使用固定的HSV参数和初始位置
    cv::Scalar hsv_mean(37.30, 181.83, 252.62);
    cv::Scalar hsv_stddev(0.57, 19.56, 1.84);
    cv::Point2d init_pos(356, 782);

    // 创建球检测器
    ball_tracker_ = std::make_unique<BallTracker>(1, "test_ball", hsv_mean, hsv_stddev, init_pos);
    
    std::cout << "HSV参数：" << std::endl;
    std::cout << "  Mean: " << hsv_mean << std::endl;
    std::cout << "  StdDev: " << hsv_stddev << std::endl;
    std::cout << "  初始位置: (" << init_pos.x << ", " << init_pos.y << ")" << std::endl;

    // 主循环
    cv::Mat frame;
    while (camera_.Capture(frame)) {
        // 更新球检测
        ball_tracker_->UpdateWithImage(frame);
        auto status = ball_tracker_->GetStatus();
        
        // 获取当前 ROI
        auto roi = ball_tracker_->GetROI();
        printf("roi: %d, %d, %d, %d\n", roi.x, roi.y, roi.width, roi.height);
        
        if (status.detected) {
            std::cout << "检测到球：" << std::endl;
            std::cout << "  ID: " << status.id << std::endl;
            std::cout << "  位置: (" << status.x << ", " << status.y << ")" << std::endl;
            std::cout << "  速度: (" << status.vx << ", " << status.vy << ")" << std::endl;
        }
    }
}

// 测试从相机读取视频并交互式选择ROI
TEST_F(BallDetectionTest, TestCameraInteractiveDetection) {
    // 初始化相机
    ASSERT_TRUE(camera_.Open("1", -1, -1, -1, CameraSourceType::USB_CAMERA))
        << "无法打开USB相机";

    cv::Mat frame;
    const std::string window_name = "Ball Detection Preview";
    cv::namedWindow(window_name, cv::WINDOW_NORMAL);
    
    // 设置鼠标回调函数
    cv::setMouseCallback(window_name, OnMouse, this);

    // 主循环
    while (true) {
        // 采集图像
        if (!camera_.Capture(frame)) {
            std::cout << "图像采集失败" << std::endl;
            continue;
        }

        cv::Mat display_frame = frame.clone();
        
        // 如果ROI已选择，计算HSV参数并初始化检测器
        if (!ball_tracker_ && roi_selected_) {
            cv::Scalar hsv_mean, hsv_stddev;
            CalculateHSVParams(frame, selected_roi_, hsv_mean, hsv_stddev);
            
            // 使用ROI中心点作为初始位置
            cv::Point2d init_pos(selected_roi_.x + selected_roi_.width/2.0,
                               selected_roi_.y + selected_roi_.height/2.0);
            ball_tracker_ = std::make_unique<BallTracker>(1, "test_ball", hsv_mean, hsv_stddev, init_pos);
            
            std::cout << "HSV参数计算完成：" << std::endl;
            std::cout << "  Mean: " << hsv_mean << std::endl;
            std::cout << "  StdDev: " << hsv_stddev << std::endl;
            std::cout << "  初始位置: (" << init_pos.x << ", " << init_pos.y << ")" << std::endl;
        }

        // 绘制ROI选择框
        if (selected_roi_.width > 0 && selected_roi_.height > 0) {
            cv::rectangle(display_frame, selected_roi_, cv::Scalar(0, 255, 0), 2);
        }

        // 如果检测器已初始化，进行球检测
        if (ball_tracker_) {
            ball_tracker_->UpdateWithImage(frame);
            auto status = ball_tracker_->GetStatus();
            
            // 获取当前 ROI
            auto roi = ball_tracker_->GetROI();
            printf("roi: %d, %d, %d, %d\n", roi.x, roi.y, roi.width, roi.height);
            
            // 绘制 ROI
            cv::rectangle(display_frame, roi, cv::Scalar(255, 0, 0), 2);  // 蓝色矩形表示 ROI
            
            if (status.detected) {
                // 在图像上绘制检测到的球
                cv::circle(display_frame, cv::Point(status.x, status.y), 10, cv::Scalar(0, 255, 0), 2);
                
                // 显示球的状态信息
                std::stringstream ss;
                ss << "ID: " << status.id 
                   << " Pos: (" << status.x << ", " << status.y << ")"
                   << " Vel: (" << status.vx << ", " << status.vy << ")";
                cv::putText(display_frame, ss.str(),
                           cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX,
                           0.7, cv::Scalar(0, 255, 0), 2);
            }
        }

        // 显示操作说明
        if (!roi_selected_) {
            cv::putText(display_frame, "Select ROI",
                       cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX,
                       0.7, cv::Scalar(0, 0, 255), 2);
        }

        cv::imshow(window_name, display_frame);
        
        // 按ESC键退出
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