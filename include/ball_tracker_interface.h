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

private:
    std::vector<std::unique_ptr<IBallTracker>> ball_trackers_;  ///< Trackers for multiple balls
    BallStatusCallback status_callback_;                        ///< Callback function for ball status updates
    bool is_tracking_ = false;                                  ///< Flag indicating if tracking is active
    std::thread tracking_thread_;                               ///< Thread for running the tracking loop
    std::mutex tracking_mutex_;                                 ///< Mutex for thread synchronization

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