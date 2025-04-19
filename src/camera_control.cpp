#include "camera_control.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <thread>
#include <mutex>
#include <chrono>

BallTrackerCamera::BallTrackerCamera()
    : width_(0)
    , height_(0)
    , fps_(0)
    , is_open_(false)
    , source_type_(CameraSourceType::USB_CAMERA)
    , dev_handle_(nullptr)
    , is_grabbing_(false)
    , dst_buffer_(nullptr)
    , dst_buffer_size_(0)
{
}

BallTrackerCamera::~BallTrackerCamera() {
    Close();
    if (dst_buffer_ != nullptr) {
        free(dst_buffer_);
        dst_buffer_ = nullptr;
    }
}

bool BallTrackerCamera::Open(const std::string& source, int width, int height, int fps, CameraSourceType source_type) {
    if (is_open_) {
        Close();
    }

    source_type_ = source_type;
    source_path_ = source;

    bool success = false;
    int ret = IMV_OK;  // 在switch语句外初始化ret

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
            // 直接使用第一个设备
            IMV_DeviceList deviceInfoList;
            ret = IMV_EnumDevices(&deviceInfoList, interfaceTypeAll);
            if (IMV_OK == ret && deviceInfoList.nDevNum > 0) {
                success = OpenByIndex(0, width, height, fps);
            }
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
        if (dev_handle_ != nullptr) {
            IMV_StopGrabbing(dev_handle_);  // 停止拉流
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
        std::cerr << "Camera is not open" << std::endl;
        return false;
    }

    if (source_type_ == CameraSourceType::HUARUI_CAMERA) {
        auto start_time = std::chrono::high_resolution_clock::now();
        
        IMV_Frame mv_frame;
        int ret = IMV_GetFrame(dev_handle_, &mv_frame, 500);
        if (IMV_OK != ret) {
            std::cerr << "Failed to get frame, error code: " << ret << std::endl;
            return false;
        }
        auto get_frame_time = std::chrono::high_resolution_clock::now();

        // 检查是否需要重新分配缓冲区
        unsigned int required_size = sizeof(unsigned char) * mv_frame.frameInfo.width * mv_frame.frameInfo.height * 3;
        if (dst_buffer_ == nullptr || dst_buffer_size_ < required_size) {
            if (dst_buffer_ != nullptr) {
                free(dst_buffer_);
            }
            dst_buffer_ = (unsigned char*)malloc(required_size);
            if (nullptr == dst_buffer_) {
                std::cerr << "Failed to allocate memory for converted image" << std::endl;
                IMV_ReleaseFrame(dev_handle_, &mv_frame);
                return false;
            }
            dst_buffer_size_ = required_size;
        }
        auto alloc_time = std::chrono::high_resolution_clock::now();

        // 设置转换参数
        IMV_PixelConvertParam stPixelConvertParam;
        memset(&stPixelConvertParam, 0, sizeof(stPixelConvertParam));
        stPixelConvertParam.nWidth = mv_frame.frameInfo.width;
        stPixelConvertParam.nHeight = mv_frame.frameInfo.height;
        stPixelConvertParam.ePixelFormat = mv_frame.frameInfo.pixelFormat;
        stPixelConvertParam.pSrcData = mv_frame.pData;
        stPixelConvertParam.nSrcDataLen = mv_frame.frameInfo.size;
        stPixelConvertParam.nPaddingX = mv_frame.frameInfo.paddingX;
        stPixelConvertParam.nPaddingY = mv_frame.frameInfo.paddingY;
        stPixelConvertParam.eBayerDemosaic = demosaicEdgeSensing;  // 使用更快的双线性插值算法
        stPixelConvertParam.eDstPixelFormat = gvspPixelBGR8;
        stPixelConvertParam.pDstBuf = dst_buffer_;
        stPixelConvertParam.nDstBufSize = dst_buffer_size_;

        // 执行转换
        ret = IMV_PixelConvert(dev_handle_, &stPixelConvertParam);
        if (IMV_OK != ret) {
            std::cerr << "Failed to convert image format, error code: " << ret << std::endl;
            IMV_ReleaseFrame(dev_handle_, &mv_frame);
            return false;
        }
        auto convert_time = std::chrono::high_resolution_clock::now();

        // 创建OpenCV图像（使用浅拷贝）
        frame = cv::Mat(mv_frame.frameInfo.height, mv_frame.frameInfo.width, 
                      CV_8UC3, dst_buffer_);
        auto create_cv_time = std::chrono::high_resolution_clock::now();

        // 释放帧数据
        IMV_ReleaseFrame(dev_handle_, &mv_frame);
        auto end_time = std::chrono::high_resolution_clock::now();

        // 输出各步骤耗时
        auto get_frame_duration = std::chrono::duration_cast<std::chrono::milliseconds>(get_frame_time - start_time);
        auto alloc_duration = std::chrono::duration_cast<std::chrono::milliseconds>(alloc_time - get_frame_time);
        auto convert_duration = std::chrono::duration_cast<std::chrono::milliseconds>(convert_time - alloc_time);
        auto create_cv_duration = std::chrono::duration_cast<std::chrono::milliseconds>(create_cv_time - convert_time);
        auto free_duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - create_cv_time);
        auto total_duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);

        std::cout << "Timing breakdown (ms): "
                  << "get_frame=" << get_frame_duration.count()
                  << ", alloc=" << alloc_duration.count()
                  << ", convert=" << convert_duration.count()
                  << ", create_cv=" << create_cv_duration.count()
                  << ", free=" << free_duration.count()
                  << ", total=" << total_duration.count() << std::endl;

        return !frame.empty();
    } else {
        return cap_.read(frame);
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

bool BallTrackerCamera::EnumHuaruiDevices(std::vector<CameraDeviceInfo>& device_list) {
    device_list.clear();
    
    IMV_DeviceList deviceInfoList;
    int ret = IMV_EnumDevices(&deviceInfoList, interfaceTypeAll);
    if (IMV_OK != ret) {
        return false;
    }

    for (unsigned int i = 0; i < deviceInfoList.nDevNum; i++) {
        IMV_DeviceInfo* pDevInfo = &deviceInfoList.pDevInfo[i];
        CameraDeviceInfo info;
        
        info.vendor_name = pDevInfo->vendorName;
        info.model_name = pDevInfo->modelName;
        info.serial_number = pDevInfo->serialNumber;
        info.camera_name = pDevInfo->cameraName;
        info.camera_type = pDevInfo->nCameraType;
        
        if (pDevInfo->nCameraType == typeGigeCamera) {
            info.ip_address = pDevInfo->DeviceSpecificInfo.gigeDeviceInfo.ipAddress;
        }
        
        device_list.push_back(info);
    }

    return true;
}

bool BallTrackerCamera::OpenByIndex(int index, int width, int height, int fps) {
    if (is_open_) {
        Close();
    }

    IMV_DeviceList deviceInfoList;
    int ret = IMV_EnumDevices(&deviceInfoList, interfaceTypeAll);
    if (IMV_OK != ret) {
        std::cerr << "Failed to enumerate devices, error code: " << ret << std::endl;
        return false;
    }

    if (index < 0 || index >= static_cast<int>(deviceInfoList.nDevNum)) {
        std::cerr << "Invalid device index: " << index << std::endl;
        return false;
    }

    // 输出设备信息
    std::cout << "Opening device: " 
              << "vendor=" << deviceInfoList.pDevInfo[index].vendorName
              << ", model=" << deviceInfoList.pDevInfo[index].modelName
              << ", serial=" << deviceInfoList.pDevInfo[index].serialNumber << std::endl;

    // 创建设备句柄
    ret = IMV_CreateHandle(&dev_handle_, modeByIndex, (void*)&index);
    if (IMV_OK != ret) {
        std::cerr << "Failed to create handle, error code: " << ret << std::endl;
        return false;
    }

    // 打开相机
    ret = IMV_Open(dev_handle_);
    if (IMV_OK != ret) {
        std::cerr << "Failed to open device, error code: " << ret << std::endl;
        IMV_DestroyHandle(dev_handle_);
        dev_handle_ = nullptr;
        return false;
    }

    // 设置图像格式为BayerRG8
    ret = IMV_SetEnumFeatureSymbol(dev_handle_, "PixelFormat", "BayerRG8");
    if (IMV_OK != ret) {
        std::cerr << "Failed to set pixel format to BayerRG8, error code: " << ret << std::endl;
        IMV_Close(dev_handle_);
        IMV_DestroyHandle(dev_handle_);
        dev_handle_ = nullptr;
        return false;
    }

    // 设置图像大小
    width = 4096;  // 固定宽度
    height = 3000; // 固定高度
    ret = IMV_SetIntFeatureValue(dev_handle_, "Width", width);
    if (IMV_OK != ret) {
        std::cerr << "Failed to set width, error code: " << ret << std::endl;
        IMV_Close(dev_handle_);
        IMV_DestroyHandle(dev_handle_);
        dev_handle_ = nullptr;
        return false;
    }

    ret = IMV_SetIntFeatureValue(dev_handle_, "Height", height);
    if (IMV_OK != ret) {
        std::cerr << "Failed to set height, error code: " << ret << std::endl;
        IMV_Close(dev_handle_);
        IMV_DestroyHandle(dev_handle_);
        dev_handle_ = nullptr;
        return false;
    }

    // 开始拉流
    ret = IMV_StartGrabbing(dev_handle_);
    if (IMV_OK != ret) {
        std::cerr << "Failed to start grabbing, error code: " << ret << std::endl;
        IMV_Close(dev_handle_);
        IMV_DestroyHandle(dev_handle_);
        dev_handle_ = nullptr;
        return false;
    }

    source_type_ = CameraSourceType::HUARUI_CAMERA;
    source_path_ = deviceInfoList.pDevInfo[index].serialNumber;
    width_ = width;
    height_ = height;
    fps_ = fps;
    is_open_ = true;

    return true;
} 