#ifndef BALL_TRACKER_COMMON_H
#define BALL_TRACKER_COMMON_H

#include <string>

// OpenCV前向声明
namespace cv {
    class Mat;
    class KalmanFilter;
    class VideoCapture;
}

/**
 * @struct BallStatus
 * @brief Stores the tracking status of an individual ball.
 */
struct BallStatus {
    int id;
    std::string color;
    double x, y;
    double vx, vy;
    double progress;
    bool detected;
};

/**
 * @struct RobotTarget
 * @brief Stores the target position and velocity for the robotic arm.
 */
struct RobotTarget {
    double X_arm, Y_arm;
    double V_arm_x, V_arm_y;
};

/**
 * @enum InitTrackErrorCode
 * @brief Error codes for track trajectory initialization.
 */
enum class InitTrackErrorCode {
    SUCCESS = 0,                      ///< Initialization successful.
    CAMERA_NOT_CONNECTED = 1,         ///< Overhead camera is not connected.
    CAMERA_CAPTURE_ERROR = 2,          ///< Camera image capture failed or invalid.
    BALL_NOT_DETECTED_AT_START = 3,   ///< Ball not detected at the initial position.
    BALL_LOST_DURING_TRACKING = 4,    ///< Ball lost during tracking.
    TRACKING_TIMEOUT = 5,             ///< Tracking process timed out.
};

/**
 * @class IBallTracker
 * @brief Interface class for ball tracking operations.
 */
class IBallTracker {
public:
    virtual ~IBallTracker() = default;

    /**
     * @brief Get current ball status.
     * @return BallStatus structure.
     */
    virtual BallStatus GetStatus() const = 0;

    /**
     * @brief Update ball tracking with new image
     * @param image Input image
     * @return true if update was successful, false otherwise
     */
    virtual bool UpdateWithImage(const cv::Mat& image) = 0;
};

#endif // BALL_TRACKER_COMMON_H