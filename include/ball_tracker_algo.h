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
    BallTracker(int ball_id, const std::string& color, const cv::Scalar& hsv_mean, const cv::Scalar& hsv_stddev, const cv::Point2d& init_pos);

    /**
     * @brief Detects and updates ball position within the given ROI image.
     * @param roi_image Image segment corresponding to the region of interest.
     * @return True if detection succeeds, false otherwise.
     */
    bool DetectAndUpdate(const cv::Mat& roi_image);

    /**
     * @brief Updates ball position using prediction when detection fails.
     */
    void PredictAndUpdate();

    /**
     * @brief Retrieves the current tracking status of the ball.
     * @return Current BallStatus structure.
     */
    BallStatus GetStatus() const override;

private:
    cv::Scalar hsv_mean_;            ///< Mean HSV values for color detection.
    cv::Scalar hsv_stddev_;          ///< HSV standard deviation for color detection.
    cv::Rect detect_roi_;            ///< Region of interest for detecting the ball.
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
    bool DetectCircle(const cv::Mat& image, cv::Point2f& center, float& radius, cv::Scalar& hsv_detected);

    /**
     * @brief Calculates the color distance between two HSV values.
     * @param hsv1 First HSV color.
     * @param hsv2 Second HSV color.
     * @return Calculated color distance.
     */
    double ColorDistance(const cv::Scalar& hsv1, const cv::Scalar& hsv2);
};

#endif // BALL_TRACKER_ALGO_H
