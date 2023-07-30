#pragma once

#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <glog/logging.h>
#include <boost/range/algorithm/min_element.hpp>

#include "../types.hpp"

namespace kal_trajectory_planner::utils {

/**
 * @brief Returns an iterator to the point on the path that is closest to the given position.
 *
 * @param path
 * @param position
 * @return Path::iterator
 */
inline Path::iterator findClosestPointOnPath(Path& path, const Position& position) {
    auto sortByDistanceLambda = [&position](const auto& first, const auto& second) {
        return (first - position).squaredNorm() < (second - position).squaredNorm();
    };
    return boost::range::min_element(path, sortByDistanceLambda);
}

/**
 * @brief Returns part of a path beginning at pathIterator and ending when the path has a length of greater the
 * minSectionLength or when the end of the path has been reached.
 *
 * @param pathSection Output: A section of the given path. Will always contain at least the point referenced by
 * sectionStart.
 * @param path Path of which a section is desired
 * @param pathIterator Iterator of path referencing the first point of the desired section
 * @param minSectionLength Minimum length of the path measured in accumulated point-to-point euclidean distance
 * @return double The accumulated distance of the computed section
 */
inline double pathSection(Path& pathSection,
                          const Path& path,
                          const Path::iterator& sectionStart,
                          const double& minSectionLength) {
    pathSection.clear();

    if (sectionStart < path.begin() || sectionStart >= path.end()) {
        throw std::out_of_range("To get a section of a path, the sectionStart must reference a point on the path!");
    }
    Path::iterator pathIterator = sectionStart;

    // Always add the first point
    pathSection.push_back(*pathIterator++);

    double distance = 0;

    while (distance < minSectionLength) {
        if (pathIterator == path.end()) {
            break;
        }

        const Position previousPosition = *(pathIterator - 1);
        const Position currentPosition = *pathIterator;

        distance += (currentPosition - previousPosition).norm();
        pathSection.push_back(*pathIterator++);
    }

    return distance;
}

/**
 * @brief Transform all points on a given path using an Isomatry2d (rotation + translation)
 *
 * @param path Input/Output: The path that is transformed
 * @param transformation The transformation applied on each point
 */
inline void transformPath(Path& path, const Eigen::Isometry2d& transformation) {
    for (auto& point : path) {
        point = transformation * point;
    }
}

/**
 * @brief Fit a polyomial to a vector of observations (2d points) using least squares
 *
 * @param points The observations
 * @param polynomialDegree The degree of the estimated polynomial
 * @return Eigen::VectorXd The n parameters of the estimated polyonmial in the order of increasing degree. The zeroth
 * element is the constant.
 */
inline Eigen::VectorXd leastSquareFitPolynomialToPoints(const std::vector<Eigen::Vector2d>& points,
                                                        const uint64_t polynomialDegree) {
    // Set up the least squares problem
    uint32_t numOfDataPoints = points.size();

    if (numOfDataPoints <= polynomialDegree) {
        throw std::invalid_argument("To estimate a polynomial of degree " + std::to_string(polynomialDegree) +
                                    ", at least " + std::to_string(polynomialDegree + 1) +
                                    " data points are required but received only " + std::to_string(numOfDataPoints) +
                                    ".");
    }

    // The observation matrix
    Eigen::MatrixXd H(numOfDataPoints, polynomialDegree + 1);

    // The observations
    Eigen::VectorXd x(numOfDataPoints);

    for (int i = 0; i < numOfDataPoints; i++) {
        for (int j = 0; j <= polynomialDegree; j++) {
            H(i, j) = std::pow(points[i].x(), j);
        }
        x(i) = points[i].y();
    }

    // Solve the least square problem by computing parameter vector y
    Eigen::VectorXd y = H.colPivHouseholderQr().solve(x);
    return y;
}


/**
 * @brief Compute a path of length numOfPoints where x is in xRange using a given polynomial
 *
 *
 * @param path Output: The computed path
 * @param polynomialParameters
 * @param xRange A tuple containing the first and the last x value to consider
 * @param numOfPoints Number of points on the computed path
 */
inline void pathFromPolynomial(Path& path,
                               const Eigen::VectorXd& polynomialParameters,
                               std::tuple<double, double> xRange,
                               const uint16_t& numOfPoints) {

    double xBegin;
    double xEnd;
    std::tie(xBegin, xEnd) = xRange;

    double deltaX = xEnd - xBegin;
    double stepWidth = deltaX / static_cast<double>(numOfPoints);

    int64_t polynomialDegree = polynomialParameters.rows() - 1;

    for (double x = xBegin; path.size() < numOfPoints; x += stepWidth) {

        // Evaluate polynomial at x
        double y = 0;
        for (int i = 0; i <= polynomialDegree; i++) {
            y += polynomialParameters(i) * std::pow(x, i);
        }

        path.emplace_back(x, y);
    }
}

/**
 * @brief Generate a trajectory from a given path. The time stamps are based on the desired speed.
 *
 * Should the desiredSpeed be 0, the time stamps will be advanced by 1s from point to point.
 *
 * @param trajectory
 * @param path
 * @param desiredSpeed
 * @param t0
 */
inline void trajectoryFromPathAndSpeed(Trajectory& trajectory,
                                       const Path& path,
                                       const double& desiredSpeed,
                                       const Time& t0) {


    Time stamp = t0;
    trajectory.clear();
    trajectory.push_back({path[0], stamp});

    for (int i = 1; i < path.size(); i++) {
        double distance = (path[i] - path[i - 1]).norm();
        Duration duration;
        if (desiredSpeed == 0) {
            duration = Duration(1);
        } else {
            duration = Duration(distance / desiredSpeed);
        }
        stamp += duration;
        trajectory.push_back({path[i], stamp});
    }
}

} // namespace kal_trajectory_planner::utils