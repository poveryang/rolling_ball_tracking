#include <sstream>

#include <opencv2/opencv.hpp>

#include "camera_control.h"

BallTrackerCamera::BallTrackerCamera()
    : width_(0)
    , height_(0)
    , fps_(0)
    , is_open_(false)
{
}

BallTrackerCamera::~BallTrackerCamera() {
    Close();
}

bool BallTrackerCamera::Open(int camera_id, int width, int height, int fps) {
    if (is_open_) {
        Close();
    }

    if (!cap_.open(camera_id)) {
        return false;
    }

    // 设置相机参数
    cap_.set(cv::CAP_PROP_FRAME_WIDTH, width);
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, height);
    cap_.set(cv::CAP_PROP_FPS, fps);

    // 验证设置是否生效
    width_ = static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_WIDTH));
    height_ = static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_HEIGHT));
    fps_ = static_cast<int>(cap_.get(cv::CAP_PROP_FPS));

    if (width_ != width || height_ != height || fps_ != fps) {
        Close();
        return false;
    }

    is_open_ = true;
    return true;
}

void BallTrackerCamera::Close() {
    if (cap_.isOpened()) {
        cap_.release();
    }
    is_open_ = false;
    width_ = 0;
    height_ = 0;
    fps_ = 0;
}

bool BallTrackerCamera::Capture(cv::Mat& frame) {
    if (!is_open_) {
        return false;
    }
    return cap_.read(frame);
}

std::string BallTrackerCamera::GetInfo() const {
    std::stringstream ss;
    ss << "Camera Info:\n"
       << "Resolution: " << width_ << "x" << height_ << "\n"
       << "FPS: " << fps_ << "\n"
       << "Status: " << (is_open_ ? "Open" : "Closed");
    return ss.str();
} 