#pragma once

#include <chrono>
#include <vector>
#include <deque>
#include <numeric>

#include <Eigen/Geometry>

namespace kal3_controller {

using Duration = std::chrono::duration<double, std::ratio<1>>;
using Time = std::chrono::time_point<std::chrono::steady_clock, Duration>;

using Position = Eigen::Vector2d;
using Path = std::vector<Position>;

using Pose = Eigen::Isometry2d;
struct StampedPose {
    Pose pose;
    Time stamp;
};

using Trajectory = std::vector<StampedPose>;

struct ControlCommand {
    struct DebugInfo {
        Position closestPointOnTrajectory;
        Position lookAheadPoint;

        double anglePath;
        double curvature;
        double errorAngle;
        double errorSignedDistance;
        double yawVehicle;
    };

    double speed;
    double steeringAngle;

    ///@brief Optional return values useful for debugging
    std::optional<DebugInfo> debugInfo;
};

struct Parameters {
    ///@brief PID parameters for punishing differences of vehicle yaw and path angle
    double kAngle_p;
    double kAngle_i;
    double kAngle_d;

    ///@brief PID parameters for punishing offset of vehicle position and planned trajectory
    double kDistance_p;
    double kDistance_i;
    double kDistance_d;

    ///@brief Parameter for longitude controller
    double k_longitude;

    ///@brief The time interval of every controller command
    double control_sample_t;

    ///@brief Index of trajectory pose used to compute path curvature and path angle
    uint16_t lookAheadIndex;

    ///@brief Velocity Threshold to stop the car
    double minVelocityThreshold;

    ///@brief Returned steering angle is limited to [-steeringAngleMax, steeringAngleMax]
    double steeringAngleMax;

    ///@brief Distance between front and rear axle of the car
    double wheelBase;
};

} // namespace kal3_controller