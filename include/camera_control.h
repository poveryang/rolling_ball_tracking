#ifndef CAMERA_CONTROL_H
#define CAMERA_CONTROL_H

#include <string>

#include <opencv2/opencv.hpp>

#include "ball_tracker_common.h"

/**
 * @enum CameraSourceType
 * @brief 定义相机输入源类型
 */
enum class CameraSourceType {
    HUARUI_CAMERA,  // 华睿相机
    USB_CAMERA,     // USB免驱相机
    VIDEO_FILE      // 视频文件
};

/**
 * @class BallTrackerCamera
 * @brief Camera control class for ball tracking system
 */
class BallTrackerCamera {
public:
    /**
     * @brief Constructor
     */
    BallTrackerCamera();

    /**
     * @brief Destructor
     */
    ~BallTrackerCamera();

    /**
     * @brief Opens and initializes the camera
     * @param source Camera device ID, video file path, or camera serial number
     * @param width Desired image width
     * @param height Desired image height
     * @param fps Desired frame rate
     * @param source_type Type of camera source
     * @return true if camera was successfully initialized, false otherwise
     */
    bool Open(const std::string& source, int width, int height, int fps, CameraSourceType source_type = CameraSourceType::USB_CAMERA);

    /**
     * @brief Closes the camera and releases resources
     */
    void Close();

    /**
     * @brief Captures a new frame from the camera
     * @param frame Output frame
     * @return true if frame was successfully captured, false otherwise
     */
    bool Capture(cv::Mat& frame);

    /**
     * @brief Gets the current camera parameters
     * @return A string containing camera information
     */
    std::string GetInfo() const;

private:
    cv::VideoCapture cap_;
    int width_;
    int height_;
    int fps_;
    bool is_open_;
    CameraSourceType source_type_;
    std::string source_path_;
    
    // 华睿相机相关成员（预留接口）
    void* huarui_camera_handle_;  // 华睿相机句柄
};

#endif // CAMERA_CONTROL_H 