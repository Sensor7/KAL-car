#pragma once

#include <Eigen/Geometry>
#include <boost/range/algorithm/min_element.hpp>

#include "../types.hpp"

namespace kal3_controller::utils {

/**
 * @brief Returns an iterator to the point on the trajectory that is closest to the given position.
 *
 * @param trajectory
 * @param position
 * @return size_t
 */
inline size_t findIndexOfClosestPointOnTrajectory(const Trajectory& trajectory, const Position& position) {
    auto sortByDistanceLambda = [&position](const auto& first, const auto& second) {
        return (first.pose.translation() - position).squaredNorm() <
               (second.pose.translation() - position).squaredNorm();
    };

    // Iterator pointing to closest point on trajectory
    auto it = boost::range::min_element(trajectory, sortByDistanceLambda);

    // Index of that iterator
    auto closestPointIndex = std::distance(trajectory.begin(), it);
    return closestPointIndex;
}

/**
 * @brief Cross product of two 2d vectors.
 *
 * @param a
 * @param b
 * @return double
 */
inline double cross2(const Eigen::Vector2d& a, const Eigen::Vector2d& b) {
    return a.x() * b.y() - b.x() * a.y();
}

/*!
 * \brief discreteCurvature calculates the approximate curvature between p1, p2 and p3.
 *  If your curve is given as a set of discrete, noise-free, and reasonably
 *	closely-spaced points, you can use the formula for a circumscribing circle
 * 	through each set of three successive points as an approximation to the
 *	curvature at the center point. The curvature of such a circle is equal to
 *	four times the area of the triangle formed by the three points, divided by
 *	the product of the triangle's three sides.
 * \param p1 the first point
 * \param p2 the second point
 * \param p3 the third point
 * \return the curvature
 */
inline double discreteCurvature(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2, const Eigen::Vector2d& p3) {
    return 2.0 * cross2(p2 - p1, p3 - p2) / ((p2 - p1).norm() * (p3 - p2).norm() * (p1 - p3).norm());
}

/**
 * @brief Compute the desired speed implicated by the time difference and euclidean distance between stamped poses.
 *
 * @param first
 * @param second
 * @return double
 */
inline double computeDesiredSpeed(const StampedPose& first, const StampedPose& second) {
    Duration timeDifference = second.stamp - first.stamp;
    double distance = (second.pose.translation() - first.pose.translation()).norm();

    return distance / timeDifference.count();
}

/**
 * @brief Returns euclidean distance of a point w.r.t. to a line with a positive sign implies the point is
 * to the left of the line and a negative sign implies the point is to the right of the line.
 *
 * The direction of the line is implied by the order of the given line points.
 *
 * @param point
 * @param line
 * @return double
 */
inline double signedDistanceBetweenPointAndLine(const Eigen::Vector2d& point,
                                                const std::tuple<Eigen::Vector2d, Eigen::Vector2d>& line) {
    Eigen::Vector2d firstPointOnLine;
    Eigen::Vector2d secondPointOnLine;

    std::tie(firstPointOnLine, secondPointOnLine) = line;
    Eigen::Vector2d firstLinePointToSecondLinePoint = secondPointOnLine - firstPointOnLine;
    Eigen::Vector2d normalVector{-firstLinePointToSecondLinePoint.y(), firstLinePointToSecondLinePoint.x()};
    normalVector.normalize();

    Eigen::Vector2d firstLinePointToPoint = point - firstPointOnLine;

    return firstLinePointToPoint.dot(normalVector);
}

/**
 * @brief Define a Pose using the 2d position and the rotation around the z axis.
 *
 * @param position
 * @param yaw
 * @return Pose
 */
inline Pose poseFromPositionandYaw(const Position& position, const double& yaw) {
    Eigen::Translation2d translation(position);
    Eigen::Rotation2Dd rotation(yaw);

    return translation * rotation;
}

/**
 * @brief Normalizes an angle in rad to be in the range of [−pi, pi]
 *
 * @param angle
 * @return double
 */
double normalizeAnglePlusMinusPi(const double& angle) {
    return std::atan2(std::sin(angle), std::cos(angle));
}

} // namespace kal_controller::utils