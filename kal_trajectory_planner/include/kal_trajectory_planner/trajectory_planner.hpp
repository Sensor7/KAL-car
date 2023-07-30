#pragma once

#include "types.hpp"

namespace kal_trajectory_planner {

/**
 * @brief An exemplary implementation of a simple trajectory planning algorithm.
 *
 * This trajectory planner will return a trajectory that follows a given path with a given (constant) velocity.
 */
class TrajectoryPlanner {
public:
    /**
     * @brief Compute a trajectory following a previously set path using a given velocity
     *
     * @param vehiclePose The current 2D vehicle pose
     * @param desiredSpeed The desired speed of the vehicle
     * @param trajectoryLength The minimum length of the returned trajectory [m]
     * @param numOfPoints The number of points the returned trajectory will have
     * @return Trajectory
     */
    Trajectory computeTrajectory(const Pose& vehiclePose,
                                 const double& desiredSpeed,
                                 const double& trajectoryLength,
                                 const uint16_t& numOfPoints);

    void setPath(const Path& path);

    void setPolynomialDegree(uint64_t polynomialDegree) {
        polynomialDegree_ = polynomialDegree;
    }
    uint64_t polynomialDegree() const {
        return polynomialDegree_;
    }

private:
    /**
     * @brief Make sure the path is set and contains enough points to compute a trajectory.
     */
    static void checkPath(const Path& path);

    /**
     * @brief Sanity check of given parameters.
     */
    static void checkParameters(const double& desiredSpeed,
                                const double& trajectoryLength,
                                const uint64_t& numOfPoints);

    ///@brief The path along which a trajectory is planned
    Path path_;

    ///@brief The degree of the polynomial used in the least square regresseion
    uint64_t polynomialDegree_ = 3;
};

} // namespace kal_trajectory_planner
