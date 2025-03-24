#include <fstream>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>

#include "ball_tracker_interface.h"
#include "ball_tracker_algo.h"


BallTrackerInterface::BallTrackerInterface(const std::string& balls_config_file_path, const std::pair<double, double>& init_pos) {
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
