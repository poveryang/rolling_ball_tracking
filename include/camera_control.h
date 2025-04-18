#ifndef CAMERA_CONTROL_H
#define CAMERA_CONTROL_H

#include <string>
#include <thread>
#include <mutex>
#include <atomic>
#include <vector>

#include <opencv2/opencv.hpp>
#include "IMV/IMVApi.h"

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

struct CameraDeviceInfo {
    std::string vendor_name;
    std::string model_name;
    std::string serial_number;
    std::string camera_name;
    std::string ip_address;  // 仅GigE相机有效
    int camera_type;  // 0: GigE, 1: U3V, 2: CL, 3: PCIe
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
     * @param width Desired image width (-1 for default)
     * @param height Desired image height (-1 for default)
     * @param fps Desired frame rate (-1 for default)
     * @param source_type Type of camera source
     * @return true if camera was successfully initialized, false otherwise
     */
    bool Open(const std::string& source, 
             int width = -1, 
             int height = -1, 
             int fps = -1, 
             CameraSourceType source_type = CameraSourceType::USB_CAMERA);

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

    // Getter methods for testing
    bool IsOpen() const { return is_open_; }
    int GetWidth() const { return width_; }
    int GetHeight() const { return height_; }
    int GetFps() const { return fps_; }

    // 枚举所有可用的华睿相机设备
    static bool EnumHuaruiDevices(std::vector<CameraDeviceInfo>& device_list);

    // 通过索引打开华睿相机
    bool OpenByIndex(int index, int width = 640, int height = 480, int fps = 30);

private:
    cv::VideoCapture cap_;
    int width_;
    int height_;
    int fps_;
    bool is_open_;
    CameraSourceType source_type_;
    std::string source_path_;
    
    // 华睿相机相关成员
    IMV_HANDLE dev_handle_;           // 相机句柄
    std::thread grab_thread_;         // 拉流线程
    std::atomic<bool> is_grabbing_;   // 拉流状态
    std::mutex frame_mutex_;          // 帧数据互斥锁
    cv::Mat current_frame_;           // 当前帧
};

#endif // CAMERA_CONTROL_H 