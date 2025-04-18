#include <iostream>
#include <string>
#include <filesystem>
#include <chrono>
#include <thread>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <opencv2/opencv.hpp>
#include "camera_control.h"

struct ImageData {
    cv::Mat frame;
    std::string filename;
    int index;
};

class ImageSaver {
public:
    ImageSaver() : stop_(false), total_images_(0), saved_images_(0) {
        saver_thread_ = std::thread(&ImageSaver::SaveThread, this);
    }

    ~ImageSaver() {
        {
            std::unique_lock<std::mutex> lock(queue_mutex_);
            stop_ = true;
        }
        queue_condition_.notify_one();
        if (saver_thread_.joinable()) {
            saver_thread_.join();
        }
    }

    void AddImage(const cv::Mat& frame, const std::string& filename, int index) {
        std::unique_lock<std::mutex> lock(queue_mutex_);
        image_queue_.push({frame.clone(), filename, index});
        total_images_++;
        lock.unlock();
        queue_condition_.notify_one();
    }

    void WaitForCompletion() {
        while (true) {
            std::unique_lock<std::mutex> lock(queue_mutex_);
            if (saved_images_ >= total_images_) {
                break;
            }
            lock.unlock();
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

private:
    void SaveThread() {
        while (true) {
            std::unique_lock<std::mutex> lock(queue_mutex_);
            queue_condition_.wait(lock, [this]() { return !image_queue_.empty() || stop_; });
            
            if (stop_ && image_queue_.empty()) {
                break;
            }

            if (!image_queue_.empty()) {
                ImageData data = std::move(image_queue_.front());
                image_queue_.pop();
                lock.unlock();

                auto start_time = std::chrono::high_resolution_clock::now();
                if (cv::imwrite(data.filename, data.frame)) {
                    auto end_time = std::chrono::high_resolution_clock::now();
                    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
                    std::cout << "Saved image " << data.index + 1 << " to " << data.filename 
                              << " (save: " << duration.count() << "ms)" << std::endl;
                    saved_images_++;
                } else {
                    std::cerr << "Failed to save image " << data.index + 1 << std::endl;
                }
            }
        }
    }

    std::queue<ImageData> image_queue_;
    std::mutex queue_mutex_;
    std::condition_variable queue_condition_;
    std::thread saver_thread_;
    bool stop_;
    std::atomic<int> total_images_;
    std::atomic<int> saved_images_;
};

int main(int argc, char** argv) {
    // 检查参数
    if (argc != 3) {
        std::cout << "Usage: " << argv[0] << " <num_images> <save_dir>" << std::endl;
        std::cout << "Example: " << argv[0] << " 60 ./test_images" << std::endl;
        return -1;
    }

    int num_images = std::stoi(argv[1]);
    std::string save_dir = argv[2];

    // 创建保存目录
    std::filesystem::create_directories(save_dir);

    // 创建相机对象
    BallTrackerCamera camera;

    // 打开相机
    if (!camera.Open("", 4096, 3000, 30, CameraSourceType::HUARUI_CAMERA)) {
        std::cerr << "Failed to open camera!" << std::endl;
        return -1;
    }

    std::cout << "Camera information:" << std::endl;
    std::cout << camera.GetInfo() << std::endl;

    // 创建图像保存器
    ImageSaver image_saver;

    // 开始采集图像
    for (int i = 0; i < num_images; ++i) {
        auto start_time = std::chrono::high_resolution_clock::now();
        
        cv::Mat frame;
        if (camera.Capture(frame)) {
            auto capture_time = std::chrono::high_resolution_clock::now();
            auto capture_duration = std::chrono::duration_cast<std::chrono::milliseconds>(capture_time - start_time);
            
            std::string filename = save_dir + "/image_" + std::to_string(i) + ".png";
            image_saver.AddImage(frame, filename, i);
            
            std::cout << "Captured image " << i + 1 << "/" << num_images 
                      << " (capture: " << capture_duration.count() << "ms)" << std::endl;
        } else {
            std::cerr << "Failed to capture image " << i + 1 << std::endl;
        }
    }

    // 等待所有图像保存完成
    std::cout << "Waiting for all images to be saved..." << std::endl;
    image_saver.WaitForCompletion();
    std::cout << "All images saved successfully." << std::endl;

    // 关闭相机
    camera.Close();
    std::cout << "Capture completed. All images saved to " << save_dir << std::endl;

    return 0;
} 