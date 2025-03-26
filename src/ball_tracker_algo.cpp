#include "ball_tracker_algo.h"

BallTracker::BallTracker(int ball_id, const std::string& color, const cv::Scalar& hsv_mean, const cv::Scalar& hsv_stddev, const cv::Point2d& init_pos) {
    // 构造函数实现
}

BallTracker::~BallTracker() {
    // 析构函数实现
}

BallStatus BallTracker::GetStatus() const {
    // 函数实现
    return ball_status_;
}