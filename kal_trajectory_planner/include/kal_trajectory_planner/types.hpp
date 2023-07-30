#pragma once

#include <chrono>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace kal_trajectory_planner {

using Duration = std::chrono::duration<double, std::ratio<1>>;
using Time = std::chrono::time_point<std::chrono::steady_clock, Duration>;

using Position = Eigen::Vector2d;
using Path = std::vector<Position>;
struct StampedPosition {
    Position position;
    Time stamp;
};
using Trajectory = std::vector<StampedPosition>;

using Pose = Eigen::Isometry2d;

} // namespace kal_trajectory_planner