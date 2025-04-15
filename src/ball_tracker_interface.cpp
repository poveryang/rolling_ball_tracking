#include <fstream>
#include <chrono>
#include <iomanip>
#include <sstream>

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
        is_initialized = camera.Open(std::to_string(camera_id), width, height, fps, CameraSourceType::USB_CAMERA);
        return is_initialized;
    }

    bool Initialize(const std::string& video_path, int width, int height, int fps) {
        is_initialized = camera.Open(video_path, width, height, fps, CameraSourceType::VIDEO_FILE);
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
    , balls_config_file_path_(balls_config_file_path)
{
    // 读取配置文件
    std::ifstream config_file(balls_config_file_path);
    if (!config_file.is_open()) {
        throw std::runtime_error("无法打开配置文件: " + balls_config_file_path);
    }

    nlohmann::json config;
    config_file >> config;

    // 创建球检测器
    for (const auto& ball_config : config["balls"]) {
        int ball_id = ball_config["id"];
        std::string color = ball_config["color"];
        cv::Scalar hsv_mean(ball_config["hsv_mean"][0], ball_config["hsv_mean"][1], ball_config["hsv_mean"][2]);
        cv::Scalar hsv_stddev(ball_config["hsv_stddev"][0], ball_config["hsv_stddev"][1], ball_config["hsv_stddev"][2]);
        
        // 使用传入的初始位置
        cv::Point2d init_pos_point(init_pos.first, init_pos.second);
        ball_trackers_.push_back(std::make_unique<BallTracker>(ball_id, color, hsv_mean, hsv_stddev, init_pos_point));
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

bool BallTrackerInterface::InitializeCamera(int camera_id, int width, int height, int fps) {
    return camera_->Initialize(camera_id, width, height, fps);
}

bool BallTrackerInterface::InitializeCamera(const std::string& video_path, int width, int height, int fps) {
    return camera_->Initialize(video_path, width, height, fps);
}

int BallTrackerInterface::InitTrack(const std::string &out_trajectory_file_path)
{
    // 检查相机是否已初始化
    if (!camera_->is_initialized) {
        // 尝试使用视频文件初始化相机
        if (!InitializeCamera("test/test_video.MOV")) {
            return static_cast<int>(InitTrackErrorCode::CAMERA_NOT_CONNECTED);
        }
    }

    // 创建轨迹数据结构
    nlohmann::json trajectory_data;
    trajectory_data["track_trajectory"]["points"] = nlohmann::json::array();
    trajectory_data["track_trajectory"]["length"] = 0.0;
    trajectory_data["track_trajectory"]["start_point"] = nlohmann::json::object();
    trajectory_data["track_trajectory"]["end_point"] = nlohmann::json::object();
    
    // 记录开始时间
    auto start_time = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(start_time);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&time_t), "%Y-%m-%d %H:%M:%S");
    trajectory_data["track_trajectory"]["timestamp"] = ss.str();

    // 记录第一个点作为起点
    bool first_point = true;
    cv::Point2d start_point;
    int consecutive_failures = 0;
    const int MAX_CONSECUTIVE_FAILURES = 200;  // 增加允许的连续失败次数
    int frame_count = 0;
    const double MAX_VELOCITY = 500.0;
    int total_frames = 0;
    int success_frames = 0;

    // 循环采集图像并记录轨迹
    while (true) {
        frame_count++;
        total_frames++;

        // 采集图像
        cv::Mat frame;
        if (!camera_->Capture(frame)) {
            std::cout << "视频文件读取结束" << std::endl;
            break;
        }

        // 只使用第一个球来更新跟踪状态
        if (!ball_trackers_.empty()) {
            bool update_success = ball_trackers_[0]->UpdateWithImage(frame);
            auto status = ball_trackers_[0]->GetStatus();
            
            // 检查检测结果是否合理
            bool is_valid = update_success;
            if (update_success) {
                // 检查速度是否在合理范围内
                double speed = std::sqrt(status.vx * status.vx + status.vy * status.vy);
                if (speed > MAX_VELOCITY) {
                    std::cout << "速度过大: " << speed << std::endl;
                    is_valid = false;
                }
            }

            if (is_valid) {
                // 记录轨迹点
                nlohmann::json point;
                point["x"] = status.x;
                point["y"] = status.y;
                trajectory_data["track_trajectory"]["points"].push_back(point);

                // 记录起点
                if (first_point) {
                    start_point = cv::Point2d(status.x, status.y);
                    trajectory_data["track_trajectory"]["start_point"]["x"] = status.x;
                    trajectory_data["track_trajectory"]["start_point"]["y"] = status.y;
                    first_point = false;
                }

                // 更新终点
                trajectory_data["track_trajectory"]["end_point"]["x"] = status.x;
                trajectory_data["track_trajectory"]["end_point"]["y"] = status.y;
                
                // 重置连续失败计数
                consecutive_failures = 0;
                success_frames++;
            } else {
                consecutive_failures++;
                std::cout << "检测失败，连续失败次数: " << consecutive_failures << std::endl;
                
                // 如果连续失败次数超过阈值，返回错误
                if (consecutive_failures >= MAX_CONSECUTIVE_FAILURES) {
                    std::cout << "连续失败次数超过阈值，终止跟踪" << std::endl;
                    // 检查成功率是否足够
                    double success_rate = static_cast<double>(success_frames) / total_frames;
                    if (success_rate < 0.5) {  // 如果成功率低于50%，返回错误
                        return static_cast<int>(InitTrackErrorCode::BALL_LOST_DURING_TRACKING);
                    } else {
                        break;  // 如果成功率足够，正常结束跟踪
                    }
                }
            }
        }
    }

    // 计算轨迹总长度
    double total_length = 0.0;
    auto& points = trajectory_data["track_trajectory"]["points"];
    for (size_t i = 1; i < points.size(); ++i) {
        double dx = points[i]["x"].get<double>() - points[i-1]["x"].get<double>();
        double dy = points[i]["y"].get<double>() - points[i-1]["y"].get<double>();
        total_length += std::sqrt(dx*dx + dy*dy);
    }
    trajectory_data["track_trajectory"]["length"] = total_length;

    // 保存轨迹数据到文件
    std::ofstream trajectory_file(out_trajectory_file_path);
    if (!trajectory_file.is_open()) {
        return static_cast<int>(InitTrackErrorCode::CAMERA_CAPTURE_ERROR);
    }
    trajectory_file << std::setw(4) << trajectory_data << std::endl;

    return static_cast<int>(InitTrackErrorCode::SUCCESS);
}

void BallTrackerInterface::StartTracking()
{
    std::lock_guard<std::mutex> lock(tracking_mutex_);
    if (is_tracking_) {
        return;  // 已经在跟踪中
    }

    // 检查相机是否已初始化
    if (!camera_->is_initialized) {
        // 尝试使用视频文件初始化相机
        if (!InitializeCamera("test/test_video.MOV")) {
            return;
        }
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

bool BallTrackerInterface::GetFirstFrame(cv::Mat& frame) {
    if (!camera_->is_initialized) {
        return false;
    }
    return camera_->Capture(frame);
}
