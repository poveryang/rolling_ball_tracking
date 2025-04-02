#include <opencv2/opencv.hpp>

#include "ball_tracker_algo.h"

BallTracker::BallTracker(int ball_id, const std::string& color, const cv::Scalar_<double>& hsv_mean, const cv::Scalar_<double>& hsv_stddev, const cv::Point_<double>& init_pos)
    : hsv_mean_(hsv_mean)
    , hsv_stddev_(hsv_stddev)
    , detect_roi_(0, 0, 100, 100)  // 初始ROI大小
    , kalman_filter_(4, 2)  // 状态向量：x, y, vx, vy；测量向量：x, y
{
    // 初始化卡尔曼滤波器
    kalman_filter_.transitionMatrix = (cv::Mat_<float>(4, 4) <<
        1, 0, 1, 0,
        0, 1, 0, 1,
        0, 0, 1, 0,
        0, 0, 0, 1);

    kalman_filter_.measurementMatrix = (cv::Mat_<float>(2, 4) <<
        1, 0, 0, 0,
        0, 1, 0, 0);

    kalman_filter_.processNoiseCov = cv::Mat::eye(4, 4, CV_32F) * 0.1;
    kalman_filter_.measurementNoiseCov = cv::Mat::eye(2, 2, CV_32F) * 0.1;
    kalman_filter_.errorCovPost = cv::Mat::eye(4, 4, CV_32F);

    // 初始化状态
    kalman_filter_.statePost.at<float>(0) = static_cast<float>(init_pos.x);
    kalman_filter_.statePost.at<float>(1) = static_cast<float>(init_pos.y);
    kalman_filter_.statePost.at<float>(2) = 0.0f;
    kalman_filter_.statePost.at<float>(3) = 0.0f;

    // 初始化球状态
    ball_status_.id = ball_id;
    ball_status_.color = color;
    ball_status_.x = init_pos.x;
    ball_status_.y = init_pos.y;
    ball_status_.vx = 0.0;
    ball_status_.vy = 0.0;
    ball_status_.progress = 0.0;
    ball_status_.detected = false;
}

BallTracker::~BallTracker() = default;

BallStatus BallTracker::GetStatus() const {
    return ball_status_;
}

bool BallTracker::UpdateWithImage(const cv::Mat& image) {
    if (image.empty()) {
        return false;
    }

    // 确保ROI在图像范围内
    detect_roi_.x = std::max(0, detect_roi_.x);
    detect_roi_.y = std::max(0, detect_roi_.y);
    detect_roi_.width = std::min(detect_roi_.width, image.cols - detect_roi_.x);
    detect_roi_.height = std::min(detect_roi_.height, image.rows - detect_roi_.y);

    // 获取ROI区域
    cv::Mat roi_image = image(detect_roi_);
    if (roi_image.empty()) {
        return false;
    }

    // 检测小球
    cv::Point2f center;
    float radius;
    cv::Scalar hsv_detected;
    bool detected = DetectCircle(roi_image, center, radius, hsv_detected);

    if (detected) {
        // 更新卡尔曼滤波器
        cv::Mat measurement = (cv::Mat_<float>(2, 1) <<
            center.x + detect_roi_.x,
            center.y + detect_roi_.y);
        kalman_filter_.correct(measurement);

        // 更新状态
        ball_status_.x = kalman_filter_.statePost.at<float>(0);
        ball_status_.y = kalman_filter_.statePost.at<float>(1);
        ball_status_.vx = kalman_filter_.statePost.at<float>(2);
        ball_status_.vy = kalman_filter_.statePost.at<float>(3);
        ball_status_.detected = true;

        // 更新ROI位置
        detect_roi_.x = static_cast<int>(ball_status_.x - radius);
        detect_roi_.y = static_cast<int>(ball_status_.y - radius);
        detect_roi_.width = static_cast<int>(radius * 2);
        detect_roi_.height = static_cast<int>(radius * 2);

        return true;
    } else {
        // 预测位置
        PredictAndUpdate();
        return false;
    }
}

void BallTracker::PredictAndUpdate() {
    // 预测
    cv::Mat prediction = kalman_filter_.predict();

    // 更新状态
    ball_status_.x = prediction.at<float>(0);
    ball_status_.y = prediction.at<float>(1);
    ball_status_.vx = prediction.at<float>(2);
    ball_status_.vy = prediction.at<float>(3);
    ball_status_.detected = false;

    // 更新ROI位置
    detect_roi_.x = static_cast<int>(ball_status_.x - 50);
    detect_roi_.y = static_cast<int>(ball_status_.y - 50);
    detect_roi_.width = 100;
    detect_roi_.height = 100;
}

bool BallTracker::DetectCircle(const cv::Mat& image, cv::Point_<float>& center, float& radius, cv::Scalar_<double>& hsv_detected) {
    // 转换为HSV颜色空间
    cv::Mat hsv_image;
    cv::cvtColor(image, hsv_image, cv::COLOR_BGR2HSV);

    // 创建颜色范围掩码
    cv::Mat mask;
    cv::Scalar lower_bound = hsv_mean_ - hsv_stddev_;
    cv::Scalar upper_bound = hsv_mean_ + hsv_stddev_;
    cv::inRange(hsv_image, lower_bound, upper_bound, mask);

    // 形态学操作
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);

    // 查找轮廓
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    if (contours.empty()) {
        return false;
    }

    // 找到最大的轮廓
    auto max_contour = std::max_element(contours.begin(), contours.end(),
        [](const auto& c1, const auto& c2) {
            return cv::contourArea(c1) < cv::contourArea(c2);
        });

    // 拟合圆
    cv::Point2f center_temp;
    float radius_temp;
    cv::minEnclosingCircle(*max_contour, center_temp, radius_temp);

    // 计算平均HSV值
    cv::Scalar mean_hsv = cv::mean(hsv_image, mask);

    // 更新输出参数
    center = center_temp;
    radius = radius_temp;
    hsv_detected = cv::Scalar_<double>(mean_hsv[0], mean_hsv[1], mean_hsv[2]);

    return true;
}

// ... 其他现有方法的实现 ...