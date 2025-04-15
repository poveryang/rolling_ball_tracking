#include <sstream>

#include <opencv2/opencv.hpp>

#include "camera_control.h"

BallTrackerCamera::BallTrackerCamera()
    : width_(0)
    , height_(0)
    , fps_(0)
    , is_open_(false)
    , source_type_(CameraSourceType::USB_CAMERA)
    , huarui_camera_handle_(nullptr)
{
}

BallTrackerCamera::~BallTrackerCamera() {
    Close();
}

bool BallTrackerCamera::Open(const std::string& source, int width, int height, int fps, CameraSourceType source_type) {
    if (is_open_) {
        Close();
    }

    source_type_ = source_type;
    source_path_ = source;

    bool success = false;
    switch (source_type) {
        case CameraSourceType::USB_CAMERA:
            try {
                int camera_id = std::stoi(source);
                if (cap_.open(camera_id)) {
                    cap_.set(cv::CAP_PROP_FRAME_WIDTH, width);
                    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, height);
                    cap_.set(cv::CAP_PROP_FPS, fps);
                    success = true;
                }
            } catch (const std::exception& e) {
                success = false;
            }
            break;

        case CameraSourceType::VIDEO_FILE:
            if (cap_.open(source)) {
                cap_.set(cv::CAP_PROP_FRAME_WIDTH, width);
                cap_.set(cv::CAP_PROP_FRAME_HEIGHT, height);
                cap_.set(cv::CAP_PROP_FPS, fps);
                success = true;
            }
            break;

        case CameraSourceType::HUARUI_CAMERA:
            // TODO: 后续实现华睿相机SDK集成
            success = false;
            break;

        default:
            success = false;
            break;
    }

    if (success) {
        // 验证设置是否生效
        if (source_type != CameraSourceType::HUARUI_CAMERA) {
            width_ = static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_WIDTH));
            height_ = static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_HEIGHT));
            fps_ = static_cast<int>(cap_.get(cv::CAP_PROP_FPS));

            if (width_ != width || height_ != height || fps_ != fps) {
                Close();
                return false;
            }
        }
        is_open_ = true;
    }

    return success;
}

void BallTrackerCamera::Close() {
    if (source_type_ == CameraSourceType::HUARUI_CAMERA) {
        // TODO: 后续实现华睿相机SDK集成
    } else if (cap_.isOpened()) {
        cap_.release();
    }
    is_open_ = false;
    width_ = 0;
    height_ = 0;
    fps_ = 0;
    source_path_.clear();
}

bool BallTrackerCamera::Capture(cv::Mat& frame) {
    if (!is_open_) {
        return false;
    }

    if (source_type_ == CameraSourceType::HUARUI_CAMERA) {
        // TODO: 后续实现华睿相机SDK集成
        return false;
    } else {
        return cap_.read(frame);
    }
}

std::string BallTrackerCamera::GetInfo() const {
    std::stringstream ss;
    ss << "Camera Info:\n"
       << "Source Type: " << (source_type_ == CameraSourceType::USB_CAMERA ? "USB Camera" : 
                             source_type_ == CameraSourceType::VIDEO_FILE ? "Video File" : "Huarui Camera") << "\n"
       << "Source: " << source_path_ << "\n"
       << "Resolution: " << width_ << "x" << height_ << "\n"
       << "FPS: " << fps_ << "\n"
       << "Status: " << (is_open_ ? "Open" : "Closed");
    return ss.str();
} 