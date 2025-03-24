#ifndef BALL_TRACKER_INTERFACE_H
#define BALL_TRACKER_INTERFACE_H

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
    ~BallTrackerInterface() = default;

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

private:
    std::vector<std::unique_ptr<IBallTracker>> ball_trackers_; ///< Trackers for multiple balls.
};

#endif // BALL_TRACKER_INTERFACE_H