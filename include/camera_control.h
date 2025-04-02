#ifndef CAMERA_CONTROL_H
#define CAMERA_CONTROL_H

#include <string>

#include <opencv2/opencv.hpp>

#include "ball_tracker_common.h"

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
     * @param camera_id Camera device ID
     * @param width Desired image width
     * @param height Desired image height
     * @param fps Desired frame rate
     * @return true if camera was successfully initialized, false otherwise
     */
    bool Open(int camera_id, int width, int height, int fps);

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
};

#endif // CAMERA_CONTROL_H 