#include <fstream>

#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>

#include "ball_tracker_interface.h"
#include "ball_tracker_algo.h"
#include "camera_control.h"

// Implementation of CameraImpl class
class BallTrackerInterface::CameraImpl {
public:
    BallTrackerCamera camera;
    bool is_initialized = false;

    bool Initialize(int camera_id, int width, int height, int fps) {
        is_initialized = camera.Open(camera_id, width, height, fps);
        return is_initialized;
    }

    void Release() {
        if (is_initialized) {
            camera.Close();
            is_initialized = false;
        }
    }

    bool Capture(cv::Mat& frame) {
        if (!is_initialized) {
            return false;
        }
        return camera.Capture(frame);
    }

    std::string GetInfo() const {
        return camera.GetInfo();
    }
};

BallTrackerInterface::BallTrackerInterface(const std::string& balls_config_file_path, const std::pair<double, double>& init_pos)
    : camera_(std::make_unique<CameraImpl>())
{
    std::ifstream file(balls_config_file_path);
    nlohmann::json config_json;
    file >> config_json;

    for (const auto& ball_config : config_json["balls"]) {
        int id = ball_config["id"];
        std::string color = ball_config["color"];

        auto hsv_mean_array = ball_config["hsv_mean"];
        auto hsv_stddev_array = ball_config["hsv_stddev"];

        cv::Scalar hsv_mean(hsv_mean_array[0], hsv_mean_array[1], hsv_mean_array[2]);
        cv::Scalar hsv_stddev(hsv_stddev_array[0], hsv_stddev_array[1], hsv_stddev_array[2]);

        cv::Point2d init_pos_cv(init_pos.first, init_pos.second);

        ball_trackers_.emplace_back(std::make_unique<BallTracker>(id, color, hsv_mean, hsv_stddev, init_pos_cv));
    }
}

BallTrackerInterface::~BallTrackerInterface() {
    StopTracking();  // 确保在析构时停止跟踪
}

void BallTrackerInterface::SetHeightParameters(const HeightParameters& heights) {
    height_params_ = heights;
    // TODO: 在这里可以添加基于高度参数的预计算逻辑
    // 例如：计算小球的像素大小等
}

int BallTrackerInterface::InitTrack(const std::string &out_trajectory_file_path)
{
    return 0;
}

void BallTrackerInterface::StartTracking()
{
    std::lock_guard<std::mutex> lock(tracking_mutex_);
    if (is_tracking_) {
        return;  // 已经在跟踪中
    }

    // 创建相机对象并打开
    BallTrackerCamera camera;
    if (!camera.Open(0, 1280, 720, 30)) {  // 参数可以根据需要调整
        return;
    }

    is_tracking_ = true;
    tracking_thread_ = std::thread(&BallTrackerInterface::TrackingLoop, this);
}

void BallTrackerInterface::StopTracking() {
    std::lock_guard<std::mutex> lock(tracking_mutex_);
    if (!is_tracking_) {
        return;  // 已经停止跟踪
    }

    is_tracking_ = false;
    if (tracking_thread_.joinable()) {
        tracking_thread_.join();
    }
}

void BallTrackerInterface::TrackingLoop() {
    // 创建相机对象并打开
    BallTrackerCamera camera;
    if (!camera.Open(0, 1280, 720, 30)) {  // 参数可以根据需要调整
        is_tracking_ = false;
        return;
    }

    // 循环跟踪直到StopTracking被调用
    while (is_tracking_) {
        // 采集图像
        cv::Mat frame;
        if (!camera_->Capture(frame)) {
            continue;  // 采集失败，继续下一帧
        }

        // 并行更新每个小球的跟踪状态
        #pragma omp parallel for
        for (int i = 0; i < ball_trackers_.size(); ++i) {
            ball_trackers_[i]->UpdateWithImage(frame);
        }
        
        // 确保所有线程都完成后再继续
        #pragma omp barrier

        // 通知回调函数
        NotifyBallStatusUpdate();
    }

    // 关闭相机
    camera.Close();
}

void BallTrackerInterface::RegisterBallStatusCallback(BallStatusCallback callback) {
    status_callback_ = std::move(callback);
}

void BallTrackerInterface::UnregisterBallStatusCallback() {
    status_callback_ = nullptr;
}

void BallTrackerInterface::NotifyBallStatusUpdate() {
    if (status_callback_) {
        status_callback_(GetBallStatus());
    }
}

std::vector<BallStatus> BallTrackerInterface::GetBallStatus() {
    std::vector<BallStatus> statuses;
    for (const auto& tracker : ball_trackers_) {
        statuses.push_back(tracker->GetStatus());
    }
    return statuses;
}

RobotTarget BallTrackerInterface::GetRobotTarget() {
    // TODO: Implement robot target position and velocity calculation
    return RobotTarget{};
}
