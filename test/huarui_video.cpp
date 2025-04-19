#include <iostream>
#include <string>
#include <filesystem>
#include <chrono>
#include <thread>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <signal.h>
#include <opencv2/opencv.hpp>
#include "camera_control.h"

std::atomic<bool> g_running(true);

void signalHandler(int signum) {
    std::cout << "Interrupt signal (" << signum << ") received.\n";
    g_running = false;
}

struct FrameData {
    cv::Mat frame;
    std::chrono::high_resolution_clock::time_point timestamp;
    int index;
};

class VideoWriter {
public:
    VideoWriter(const std::string& filename, int width, int height, int target_fps)
        : stop_(false), total_frames_(0), written_frames_(0), target_fps_(target_fps) {
        // 使用H.264编码器
        int fourcc = cv::VideoWriter::fourcc('H', '2', '6', '4');
        writer_ = std::make_unique<cv::VideoWriter>(
            filename,
            fourcc,
            target_fps,
            cv::Size(width, height)
        );
        if (!writer_->isOpened()) {
            throw std::runtime_error("Could not open video writer");
        }
        writer_thread_ = std::thread(&VideoWriter::WriteThread, this);
        start_time_ = std::chrono::high_resolution_clock::now();
    }

    ~VideoWriter() {
        {
            std::unique_lock<std::mutex> lock(queue_mutex_);
            stop_ = true;
        }
        queue_condition_.notify_one();
        if (writer_thread_.joinable()) {
            writer_thread_.join();
        }
        writer_->release();
    }

    void AddFrame(const cv::Mat& frame, int index) {
        std::unique_lock<std::mutex> lock(queue_mutex_);
        frame_queue_.push({frame.clone(), std::chrono::high_resolution_clock::now(), index});
        total_frames_++;
        lock.unlock();
        queue_condition_.notify_one();
    }

    void WaitForCompletion() {
        while (true) {
            std::unique_lock<std::mutex> lock(queue_mutex_);
            if (written_frames_ >= total_frames_ && frame_queue_.empty()) {
                break;
            }
            lock.unlock();
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

private:
    void WriteThread() {
        std::chrono::high_resolution_clock::time_point last_frame_time;
        bool is_first_frame = true;

        while (true) {
            std::unique_lock<std::mutex> lock(queue_mutex_);
            queue_condition_.wait(lock, [this]() { return !frame_queue_.empty() || stop_; });
            
            if (stop_ && frame_queue_.empty()) {
                break;
            }

            if (!frame_queue_.empty()) {
                FrameData data = std::move(frame_queue_.front());
                frame_queue_.pop();
                lock.unlock();

                // 计算实际帧间隔
                if (!is_first_frame) {
                    auto frame_interval = std::chrono::duration_cast<std::chrono::milliseconds>(
                        data.timestamp - last_frame_time
                    ).count();
                    
                    // 如果帧间隔太小，等待一段时间
                    if (frame_interval < 1000.0 / target_fps_) {
                        std::this_thread::sleep_for(
                            std::chrono::milliseconds(static_cast<int>(1000.0 / target_fps_ - frame_interval))
                        );
                    }
                } else {
                    is_first_frame = false;
                }

                writer_->write(data.frame);
                written_frames_++;
                last_frame_time = data.timestamp;
            }
        }
    }

    std::queue<FrameData> frame_queue_;
    std::mutex queue_mutex_;
    std::condition_variable queue_condition_;
    std::unique_ptr<cv::VideoWriter> writer_;
    std::thread writer_thread_;
    bool stop_;
    std::atomic<int> total_frames_;
    std::atomic<int> written_frames_;
    int target_fps_;
    std::chrono::high_resolution_clock::time_point start_time_;
};

int main(int argc, char** argv) {
    // 注册信号处理
    signal(SIGINT, signalHandler);

    // 检查参数
    if (argc != 2) {
        std::cout << "Usage: " << argv[0] << " <output_video>" << std::endl;
        std::cout << "Example: " << argv[0] << " output.avi" << std::endl;
        return -1;
    }

    std::string output_file = argv[1];

    // 检查输出文件扩展名
    std::filesystem::path output_path(output_file);
    if (output_path.extension() != ".mp4") {
        output_path.replace_extension(".mp4");
        output_file = output_path.string();
        std::cout << "Output file extension changed to .mp4" << std::endl;
    }

    // 创建相机对象
    BallTrackerCamera camera;

    // 打开相机
    if (!camera.Open("", 4096, 3000, 30, CameraSourceType::HUARUI_CAMERA)) {
        std::cerr << "Failed to open camera!" << std::endl;
        return -1;
    }

    std::cout << "Camera information:" << std::endl;
    std::cout << camera.GetInfo() << std::endl;

    // 创建视频写入器
    VideoWriter writer(output_file, 4096, 3000, 30);

    std::cout << "Recording started. Press Ctrl+C to stop." << std::endl;
    
    int frame_count = 0;
    auto start_time = std::chrono::high_resolution_clock::now();

    // 主循环
    while (g_running) {
        auto frame_start = std::chrono::high_resolution_clock::now();
        
        cv::Mat frame;
        if (camera.Capture(frame)) {
            auto capture_time = std::chrono::high_resolution_clock::now();
            auto capture_duration = std::chrono::duration_cast<std::chrono::milliseconds>(capture_time - frame_start);
            
            writer.AddFrame(frame, frame_count);
            frame_count++;
            
            // 显示当前帧数和采集时间
            std::cout << "\rFrames: " << frame_count 
                      << ", Capture time: " << capture_duration.count() << "ms\n" << std::flush;
        } else {
            std::cerr << "Failed to capture frame" << std::endl;
            break;
        }
    }

    std::cout << "\nStopping recording..." << std::endl;
    
    // 等待所有帧写入完成
    writer.WaitForCompletion();
    
    // 关闭相机
    camera.Close();
    
    // 计算总时长和视频时长
    auto end_time = std::chrono::high_resolution_clock::now();
    auto total_duration = std::chrono::duration_cast<std::chrono::seconds>(end_time - start_time).count();
    double video_duration = frame_count / 30.0;  // 30fps
    
    std::cout << "Recording completed. Total frames: " << frame_count << std::endl;
    std::cout << "Total duration: " << total_duration << " seconds" << std::endl;
    std::cout << "Video duration: " << std::fixed << std::setprecision(2) << video_duration << " seconds" << std::endl;
    std::cout << "Video saved to: " << output_file << std::endl;

    return 0;
} 