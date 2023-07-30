#include "trajectory_planner.hpp"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <tuple>

#include <Eigen/Geometry>
#include <glog/logging.h>

#include "types.hpp"
#include "internal/utils.hpp"

namespace kal_trajectory_planner {

Trajectory TrajectoryPlanner::computeTrajectory(const Pose& vehiclePose,
                                                const double& desiredSpeed,
                                                const double& trajectoryLength,
                                                const uint16_t& numOfPoints) {

    // Sanity checks
    checkPath(path_);
    checkParameters(desiredSpeed, trajectoryLength, numOfPoints);

    // Find closest point on the path to start planning the trajectory from there
    auto clostestPointOnPathIterator = utils::findClosestPointOnPath(path_, vehiclePose.translation());

    // Extract enough points out of the path such that the accumulated distance between these points is at least as long
    // as trajectoryLength
    Path pointsToFit;
    double distance = utils::pathSection(pointsToFit, path_, clostestPointOnPathIterator, trajectoryLength);

    // Reduce speed if the path section is shorter than desired
    double plannedSpeed = std::min(desiredSpeed, distance / trajectoryLength * desiredSpeed);

    // Reduce the polynomial degree if there are not enough points left to estimate the desired degree
    uint64_t polynomialDegree = std::min(polynomialDegree_, pointsToFit.size() - 1);

    // Estimation of the polynomial should happen in another coordinate system where points are stretched out along the
    // x-axis as much as possible
    Eigen::Vector2d connectingVector = pointsToFit.back() - pointsToFit.front();
    double angleOfConnectingVector = std::atan2(connectingVector.y(), connectingVector.x());
    Eigen::Isometry2d estimationCoordinateSystem =
        Eigen::Translation2d(vehiclePose.translation()) * Eigen::Rotation2Dd(angleOfConnectingVector);
    utils::transformPath(pointsToFit, estimationCoordinateSystem.inverse());

    // Estimate the polynomial parameters using least square estimation
    Eigen::VectorXd polynomialParameters = utils::leastSquareFitPolynomialToPoints(pointsToFit, polynomialDegree);

    // Compute path on estimated polynomial
    Path path;
    double xFront = pointsToFit.front().x();
    double xBack = pointsToFit.back().x();
    utils::pathFromPolynomial(path, polynomialParameters, std::make_tuple(xFront, xBack), numOfPoints);
    utils::transformPath(path, estimationCoordinateSystem);

    // Compute trajectory by setting time stamps depending on the planned speed
    Trajectory trajectory;
    Time t0 = std::chrono::steady_clock::now();
    utils::trajectoryFromPathAndSpeed(trajectory, path, plannedSpeed, t0);

    return trajectory;
}

void TrajectoryPlanner::setPath(const Path& path) {
    checkPath(path);

    path_ = path;
}

void TrajectoryPlanner::checkPath(const Path& path) {
    if (path.empty()) {
        throw std::runtime_error("Path must contain points!");
    }
}

void TrajectoryPlanner::checkParameters(const double& desiredSpeed,
                                        const double& trajectoryLength,
                                        const uint64_t& numOfPoints) {
    if (desiredSpeed <= 0) {
        throw std::invalid_argument("Desired speed must be greater than 0!");
    }
    if (trajectoryLength <= 0) {
        throw std::invalid_argument("Trajectory length must be greater than 0!");
    }
    if (numOfPoints <= 1) {
        throw std::invalid_argument("Output trajectory must have at least two points.");
    }
}

} // namespace kal_trajectory_planner
