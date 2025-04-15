#ifndef BALL_TRACKER_ALGO_H
#define BALL_TRACKER_ALGO_H

#include <opencv2/opencv.hpp>

#include "ball_tracker_common.h"

/**
 * @class BallTracker
 * @brief Concrete implementation of IBallTracker.
 */
class BallTracker final : public IBallTracker {
public:
    /**
     * @brief Constructs a BallTracker with specified parameters.
     * @param ball_id Unique identifier for the ball.
     * @param color Color description of the ball.
     * @param hsv_mean Mean HSV color values for detection.
     * @param hsv_stddev Standard deviation of HSV color values for detection.
     * @param init_pos Initial position to start tracking from.
     */
    BallTracker(int ball_id, const std::string& color, const cv::Scalar_<double>& hsv_mean, const cv::Scalar_<double>& hsv_stddev, const cv::Point_<double>& init_pos);

    /**
     * @brief Destructor for BallTracker.
     */
    ~BallTracker();

    /**
     * @brief Get current ball status.
     * @return BallStatus structure.
     */
    [[nodiscard]] BallStatus GetStatus() const override;

    /**
     * @brief Update ball tracking with new image
     * @param image Input image
     * @return true if update was successful, false otherwise
     */
    bool UpdateWithImage(const cv::Mat& image) override;

    /**
     * @brief Get the region of interest (ROI) for the ball tracker.
     * @return The region of interest as a cv::Rect.
     */
    cv::Rect GetROI() const { return detect_roi_; }

private:
    cv::Scalar_<double> hsv_mean_;            ///< Mean HSV values for color detection.
    cv::Scalar_<double> hsv_stddev_;          ///< HSV standard deviation for color detection.
    cv::Point_<double> init_pos_;            ///< Initial position to start tracking from.
    cv::Rect_<int> detect_roi_;            ///< Region of interest for detecting the ball.
    cv::KalmanFilter kalman_filter_; ///< Kalman filter instance for tracking.
    BallStatus ball_status_;         ///< Current ball status data.

    /**
     * @brief Detects a circular shape within the image.
     * @param image Input image to detect circles in.
     * @param center Detected circle center output.
     * @param radius Detected circle radius output.
     * @param hsv_detected Detected HSV color output.
     * @return True if circle is detected, false otherwise.
     */
    bool DetectCircle(const cv::Mat& image, cv::Point_<float>& center, float& radius, cv::Scalar_<double>& hsv_detected);

    /**
     * @brief Calculates the color distance between two HSV values.
     * @param hsv1 First HSV color.
     * @param hsv2 Second HSV color.
     * @return Calculated color distance.
     */
    double ColorDistance(const cv::Scalar_<double>& hsv1, const cv::Scalar_<double>& hsv2);

    /**
     * @brief Updates ball position using prediction when detection fails.
     */
    void PredictAndUpdate();
};

#endif // BALL_TRACKER_ALGO_H
