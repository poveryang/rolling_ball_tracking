#include <opencv2/opencv.hpp>

#include "ball_tracker_algo.h"

BallTracker::BallTracker(int ball_id, const std::string& color, const cv::Scalar_<double>& hsv_mean, const cv::Scalar_<double>& hsv_stddev, const cv::Point_<double>& init_pos)
    : hsv_mean_(hsv_mean)
    , hsv_stddev_(hsv_stddev)
    , init_pos_(init_pos)
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

    // 设置过程噪声和测量噪声
    kalman_filter_.processNoiseCov = cv::Mat::eye(4, 4, CV_32F) * 0.01;
    kalman_filter_.measurementNoiseCov = cv::Mat::eye(2, 2, CV_32F) * 0.1;
    kalman_filter_.errorCovPost = cv::Mat::eye(4, 4, CV_32F) * 0.1;

    // 初始化状态
    kalman_filter_.statePost.at<float>(0) = static_cast<float>(init_pos.x);
    kalman_filter_.statePost.at<float>(1) = static_cast<float>(init_pos.y);
    kalman_filter_.statePost.at<float>(2) = 0.0f;
    kalman_filter_.statePost.at<float>(3) = 0.0f;

    printf("Kalman init: pos=(%f, %f)\n", 
           kalman_filter_.statePost.at<float>(0),
           kalman_filter_.statePost.at<float>(1));

    // 初始化球状态
    ball_status_.id = ball_id;
    ball_status_.color = color;
    ball_status_.x = init_pos.x;
    ball_status_.y = init_pos.y;
    ball_status_.vx = 0.0;
    ball_status_.vy = 0.0;
    ball_status_.progress = 0.0;
    ball_status_.detected = false;

    // 只设置 ROI 的初始位置
    detect_roi_.x = static_cast<int>(init_pos.x);
    detect_roi_.y = static_cast<int>(init_pos.y);
    printf("Constructor: init_pos=(%f, %f), roi=(%d, %d)\n", 
           init_pos.x, init_pos.y, 
           detect_roi_.x, detect_roi_.y);
}

BallTracker::~BallTracker() = default;

BallStatus BallTracker::GetStatus() const {
    return ball_status_;
}

bool BallTracker::UpdateWithImage(const cv::Mat& image) {
    if (image.empty()) {
        return false;
    }

    // 如果是第一次检测，只设置 ROI 的大小
    if (detect_roi_.width == 0 || detect_roi_.height == 0) {
        int roi_size = static_cast<int>(std::min(image.cols, image.rows) / 2.0);
        detect_roi_.width = roi_size;
        detect_roi_.height = roi_size;
        detect_roi_.x = static_cast<int>(init_pos_.x - static_cast<double>(roi_size)/2.0);
        detect_roi_.y = static_cast<int>(init_pos_.y - static_cast<double>(roi_size)/2.0);
    }

    // 如果 ROI 超出图像范围，重置为全图
    if (detect_roi_.x < 0 || detect_roi_.y < 0 || 
        detect_roi_.x >= image.cols || detect_roi_.y >= image.rows) {
        detect_roi_.width = image.cols;
        detect_roi_.height = image.rows;
        detect_roi_.x = 0;
        detect_roi_.y = 0;
        
        // 重置卡尔曼滤波器状态
        kalman_filter_.statePost.at<float>(0) = static_cast<float>(image.cols / 2);
        kalman_filter_.statePost.at<float>(1) = static_cast<float>(image.rows / 2);
        kalman_filter_.statePost.at<float>(2) = 0.0f;
        kalman_filter_.statePost.at<float>(3) = 0.0f;
        
        printf("ROI reset to full image: roi=(%d, %d, %d, %d)\n",
               detect_roi_.x, detect_roi_.y,
               detect_roi_.width, detect_roi_.height);
    }

    // 确保ROI在图像范围内
    detect_roi_.x = std::max(0, std::min(detect_roi_.x, image.cols - 1));
    detect_roi_.y = std::max(0, std::min(detect_roi_.y, image.rows - 1));
    detect_roi_.width = std::min(detect_roi_.width, image.cols - detect_roi_.x);
    detect_roi_.height = std::min(detect_roi_.height, image.rows - detect_roi_.y);

    // 如果ROI无效，重置为全图
    if (detect_roi_.width <= 0 || detect_roi_.height <= 0) {
        detect_roi_.width = image.cols;
        detect_roi_.height = image.rows;
        detect_roi_.x = 0;
        detect_roi_.y = 0;
    }

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
        // 将 ROI 局部坐标转换为全局坐标
        float global_x = center.x + detect_roi_.x;
        float global_y = center.y + detect_roi_.y;
        
        // 直接使用检测结果更新球状态
        ball_status_.x = global_x;
        ball_status_.y = global_y;
        ball_status_.detected = true;

        // 更新卡尔曼滤波器的状态（但不使用其预测结果）
        cv::Mat measurement = (cv::Mat_<float>(2, 1) << global_x, global_y);
        kalman_filter_.correct(measurement);

        // 更新ROI位置和大小（以球为中心，大小为球直径的2倍）
        int new_size = static_cast<int>(radius * 4);  // 2倍直径
        detect_roi_.x = static_cast<int>(global_x - static_cast<double>(new_size)/2.0);
        detect_roi_.y = static_cast<int>(global_y - static_cast<double>(new_size)/2.0);
        detect_roi_.width = new_size;
        detect_roi_.height = new_size;
        
        return true;
    } else {
        // 检测失败时，使用卡尔曼滤波预测
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

    // 扩大ROI
    int new_width = detect_roi_.width * 2;
    int new_height = detect_roi_.height * 2;
    
    // 计算新的ROI位置
    detect_roi_.x = static_cast<int>(ball_status_.x - static_cast<double>(new_width)/2.0);
    detect_roi_.y = static_cast<int>(ball_status_.y - static_cast<double>(new_height)/2.0);
    detect_roi_.width = new_width;
    detect_roi_.height = new_height;
    
    printf("Predict: pos=(%f, %f), roi=(%d, %d, %d, %d)\n",
           ball_status_.x, ball_status_.y,
           detect_roi_.x, detect_roi_.y,
           detect_roi_.width, detect_roi_.height);
}

bool BallTracker::DetectCircle(const cv::Mat& image, cv::Point_<float>& center, float& radius, cv::Scalar_<double>& hsv_detected) {
    // 转换为HSV颜色空间
    cv::Mat hsv_image;
    cv::cvtColor(image, hsv_image, cv::COLOR_BGR2HSV);

    // 创建颜色范围掩码
    cv::Mat mask;
    cv::Scalar lower_bound = hsv_mean_ - hsv_stddev_ * 2.0;  // 扩大颜色范围
    cv::Scalar upper_bound = hsv_mean_ + hsv_stddev_ * 2.0;
    cv::inRange(hsv_image, lower_bound, upper_bound, mask);

    // 形态学操作
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);

    // 查找轮廓
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    if (contours.empty()) {
        printf("No contours found\n");
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

    printf("Detected: center=(%f, %f), radius=%f, hsv=(%f, %f, %f)\n",
           center.x, center.y, radius,
           hsv_detected[0], hsv_detected[1], hsv_detected[2]);

    return true;
}

// ... 其他现有方法的实现 ...