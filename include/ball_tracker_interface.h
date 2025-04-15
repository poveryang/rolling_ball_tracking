#ifndef BALL_TRACKER_INTERFACE_H
#define BALL_TRACKER_INTERFACE_H

#include <vector>
#include <memory>
#include <functional>
#include <string>
#include <thread>
#include <mutex>
#include <omp.h>

#include "ball_tracker_common.h"

/**
 * @struct HeightParameters
 * @brief Structure to store height-related parameters for ball tracking
 */
struct HeightParameters {
    double camera_height;      ///< Height of the camera from the ground
    double track_start_height; ///< Height of the track starting point
    double track_end_height;   ///< Height of the track ending point
};

/**
 * @class BallTrackerInterface
 * @brief Interface to manage ball tracking, trajectory initialization, and robot target acquisition.
 */
class BallTrackerInterface {
public:
    /**
     * @brief Constructor that initializes ball trackers from a configuration file and initial position.
     * @param balls_config_file_path Path to the JSON configuration file for balls.
     * @param init_pos Initial starting position for all ball trackers.
     */
    explicit BallTrackerInterface(const std::string& balls_config_file_path, const std::pair<double, double>& init_pos);

    /**
     * @brief Destructor.
     */
    ~BallTrackerInterface();

    /**
     * @brief Get the first frame from the camera
     * @param frame Output frame
     * @return Whether the frame was successfully captured
     */
    bool GetFirstFrame(cv::Mat& frame);

    /**
     * @brief Initializes the track trajectory by running one ball around the track.
     * @param out_trajectory_file_path File path to save the generated trajectory data.
     * @return Initialization status code defined by InitTrackErrorCode.
     */
    int InitTrack(const std::string& out_trajectory_file_path);

    /**
     * @brief Starts the ball tracking algorithm upon receiving the start signal.
     */
    void StartTracking();

    /**
     * @brief Stops the ball tracking algorithm upon receiving the stop signal.
     */
    void StopTracking();

    /**
     * @brief Retrieves the status of all tracked balls.
     * @return Vector containing the status of each ball.
     */
    std::vector<BallStatus> GetBallStatus();

    /**
     * @brief Retrieves the current target position and velocity for the robotic arm.
     * @return RobotTarget structure containing position and velocity data.
     */
    RobotTarget GetRobotTarget();

    /**
     * @brief Callback function type for ball status updates
     */
    using BallStatusCallback = std::function<void(const std::vector<BallStatus>&)>;

    /**
     * @brief Registers a callback function to be called when ball status is updated
     * @param callback The callback function to register
     */
    void RegisterBallStatusCallback(BallStatusCallback callback);

    /**
     * @brief Removes the registered callback function
     */
    void UnregisterBallStatusCallback();

    /**
     * @brief Sets the height parameters for the ball tracking system
     * @param heights Height parameters including camera height and track heights
     */
    void SetHeightParameters(const HeightParameters& heights);

    /**
     * @brief Initialize USB camera
     * @param camera_id Camera ID
     * @param width Image width
     * @param height Image height
     * @param fps Frame rate
     * @return Whether initialization was successful
     */
    bool InitializeCamera(int camera_id, int width = -1, int height = -1, int fps = -1);

    /**
     * @brief Initialize video file as input source
     * @param video_path Path to video file
     * @param width Image width
     * @param height Image height
     * @param fps Frame rate
     * @return Whether initialization was successful
     */
    bool InitializeCamera(const std::string& video_path, int width = -1, int height = -1, int fps = -1);

private:
    std::vector<std::unique_ptr<IBallTracker>> ball_trackers_;  ///< Trackers for multiple balls
    HeightParameters height_params_;                             ///< Height parameters for the system
    std::string balls_config_file_path_;                        ///< Path to the balls configuration file

    bool is_tracking_ = false;                                  ///< Flag indicating if tracking is active
    std::thread tracking_thread_;                               ///< Thread for running the tracking loop
    std::mutex tracking_mutex_;                                 ///< Mutex for thread synchronization

    BallStatusCallback status_callback_;                        ///< Callback function for ball status updates

    class CameraImpl;                                           ///< Forward declaration of camera implementation
    std::unique_ptr<CameraImpl> camera_;                        ///< Camera implementation using PIMPL pattern

    /**
     * @brief Calls the registered callback function with current ball status
     */
    void NotifyBallStatusUpdate();

    /**
     * @brief Main tracking loop that runs in a separate thread
     */
    void TrackingLoop();
};

#endif  // BALL_TRACKER_INTERFACE_H