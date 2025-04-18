#include <sstream>

#include <opencv2/opencv.hpp>

#include "camera_control.h"

BallTrackerCamera::BallTrackerCamera()
    : width_(0)
    , height_(0)
    , fps_(0)
    , is_open_(false)
    , source_type_(CameraSourceType::USB_CAMERA)
    , dev_handle_(nullptr)
    , is_grabbing_(false)
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
                    if (width > 0) cap_.set(cv::CAP_PROP_FRAME_WIDTH, width);
                    if (height > 0) cap_.set(cv::CAP_PROP_FRAME_HEIGHT, height);
                    if (fps > 0) cap_.set(cv::CAP_PROP_FPS, fps);
                    success = true;
                }
            } catch (const std::exception& e) {
                success = false;
            }
            break;

        case CameraSourceType::VIDEO_FILE:
            if (cap_.open(source)) {
                if (width > 0) cap_.set(cv::CAP_PROP_FRAME_WIDTH, width);
                if (height > 0) cap_.set(cv::CAP_PROP_FRAME_HEIGHT, height);
                if (fps > 0) cap_.set(cv::CAP_PROP_FPS, fps);
                success = true;
            }
            break;

        case CameraSourceType::HUARUI_CAMERA:
            success = InitHuaruiCamera(source);
            break;

        default:
            success = false;
            break;
    }

    if (success) {
        if (source_type != CameraSourceType::HUARUI_CAMERA) {
            width_ = static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_WIDTH));
            height_ = static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_HEIGHT));
            fps_ = static_cast<int>(cap_.get(cv::CAP_PROP_FPS));

            if ((width > 0 && width_ != width) || 
                (height > 0 && height_ != height) || 
                (fps > 0 && fps_ != fps)) {
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
        StopGrabbing();
        if (dev_handle_ != nullptr) {
            IMV_Close(dev_handle_);
            IMV_DestroyHandle(dev_handle_);
            dev_handle_ = nullptr;
        }
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
        std::lock_guard<std::mutex> lock(frame_mutex_);
        if (!current_frame_.empty()) {
            frame = current_frame_.clone();
            return true;
        }
        return false;
    } else {
        return cap_.read(frame);
    }
}

bool BallTrackerCamera::InitHuaruiCamera(const std::string& serial_number) {
    int ret = IMV_OK;
    
    // 发现设备
    IMV_DeviceList deviceInfoList;
    ret = IMV_EnumDevices(&deviceInfoList, interfaceTypeAll);
    if (IMV_OK != ret) {
        return false;
    }

    // 根据序列号查找设备
    int cameraIndex = -1;
    for (unsigned int i = 0; i < deviceInfoList.nDevNum; i++) {
        if (strcmp(deviceInfoList.pDevInfo[i].serialNumber, serial_number.c_str()) == 0) {
            cameraIndex = i;
            break;
        }
    }

    if (cameraIndex == -1) {
        return false;
    }

    // 创建设备句柄
    ret = IMV_CreateHandle(&dev_handle_, modeByIndex, (void*)&cameraIndex);
    if (IMV_OK != ret) {
        return false;
    }

    // 打开相机
    ret = IMV_Open(dev_handle_);
    if (IMV_OK != ret) {
        IMV_DestroyHandle(dev_handle_);
        dev_handle_ = nullptr;
        return false;
    }

    // 开始拉流
    ret = IMV_StartGrabbing(dev_handle_);
    if (IMV_OK != ret) {
        IMV_Close(dev_handle_);
        IMV_DestroyHandle(dev_handle_);
        dev_handle_ = nullptr;
        return false;
    }

    // 创建拉流线程
    is_grabbing_ = true;
    grab_thread_ = std::thread(&BallTrackerCamera::GrabThreadFunc, this);

    return true;
}

void BallTrackerCamera::GrabThreadFunc() {
    IMV_Frame frame;
    cv::Mat cv_frame;

    while (is_grabbing_) {
        int ret = IMV_GetFrame(dev_handle_, &frame, 500);
        if (IMV_OK != ret) {
            continue;
        }

        // 将帧数据转换为OpenCV格式
        cv_frame = cv::Mat(frame.frameInfo.height, frame.frameInfo.width, 
                          CV_8UC1, frame.pData);

        {
            std::lock_guard<std::mutex> lock(frame_mutex_);
            current_frame_ = cv_frame.clone();
        }

        // 释放帧数据
        IMV_ReleaseFrame(dev_handle_, &frame);
    }
}

void BallTrackerCamera::StopGrabbing() {
    if (is_grabbing_) {
        is_grabbing_ = false;
        if (grab_thread_.joinable()) {
            grab_thread_.join();
        }
        IMV_StopGrabbing(dev_handle_);
    }
}

std::string BallTrackerCamera::GetInfo() const {
    std::stringstream ss;
    ss << "Camera Info:\n"
       << "  Source Type: " << (source_type_ == CameraSourceType::USB_CAMERA ? "USB Camera" : 
                               source_type_ == CameraSourceType::VIDEO_FILE ? "Video File" : "Huarui Camera") << "\n"
       << "  Source: " << source_path_ << "\n"
       << "  Resolution: " << width_ << "x" << height_ << "\n"
       << "  FPS: " << fps_ << "\n"
       << "  Status: " << (is_open_ ? "Open" : "Closed");
    return ss.str();
} 