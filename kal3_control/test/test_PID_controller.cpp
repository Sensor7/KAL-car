// google test docs
// wiki page: https://code.google.com/p/googletest/w/list
// primer: https://code.google.com/p/googletest/wiki/V1_7_Primer
// FAQ: https://code.google.com/p/googletest/wiki/FAQ
// advanced guide: https://code.google.com/p/googletest/wiki/V1_7_AdvancedGuide
// samples: https://code.google.com/p/googletest/wiki/V1_7_Samples
//
#include "kal3_controller/controller.hpp"

#include <cstdint>
#include <stdexcept>

#include <glog/logging.h>
#include "gtest/gtest.h"

#include "kal3_controller/types.hpp"
#include "kal3_controller/internal/utils.hpp"

namespace kal3_controller {

/**
 * @brief Create a mocked trajectory using equidistant x-positions and a defined polynomial
 *
 * @param trajectoryLength
 * @param xStart
 * @param xStep
 * @param desiredSpeed
 * @return Trajectory
 */
Trajectory mockTrajectory(uint16_t trajectoryLength, double xStart, double xStep, double desiredSpeed) {
    Trajectory trajectory;
    trajectory.reserve(trajectoryLength);

    std::vector<Position> positions;

    double x = xStart;
    while (positions.size() < trajectoryLength) {

        // Compute corresponding y values using a 3rd degree polynomial
        double y = 0.007 * x * x * x - 0.1 * x * x + 0.75 * x;

        // Store position
        positions.emplace_back(x, y);

        // Move x
        x += xStep;
    }

    for (int i = 1; i < positions.size() - 1; i++) {
        Eigen::Vector2d vectorPointingToNextPosition = positions[i + 1] - positions[i];
        double yaw = std::atan2(vectorPointingToNextPosition.y(), vectorPointingToNextPosition.x());

        // Compute time stamp depending on travelled distance and desired speed
        double distanceTravelled = (positions[i] - positions[1]).norm();
        Duration duration =
            std::chrono::duration_cast<Duration>(std::chrono::duration<double>(distanceTravelled / desiredSpeed));
        Time stamp(duration);

        trajectory.push_back({utils::poseFromPositionandYaw(positions[i], yaw), stamp});
    }

    return trajectory;
}

class ControllerTest : public testing::Test {
protected:
    void SetUp() override {

        parameters_.kAngle_p = 2;
        parameters_.kAngle_i = 0;
        parameters_.kAngle_d = 0.2;
        parameters_.kDistance_p = 2;
        parameters_.kDistance_i = 0;
        parameters_.kDistance_d = 0.2;

        parameters_.k_longitude = 0;

        parameters_.control_sample_t = 0.1;

        parameters_.lookAheadIndex = 5;
        parameters_.minVelocityThreshold = 0.2;
        parameters_.steeringAngleMax = 0.3;
        parameters_.wheelBase = 0.3;

        controller_.setParameters(parameters_);

        uint16_t trajectoryLength = 20;
        double xStart = -1;
        double xStep = 1;

        trajectory_ = mockTrajectory(trajectoryLength, xStart, xStep, desiredSpeed_);
    }

    Controller controller_;
    Parameters parameters_;
    Trajectory trajectory_;

    double desiredSpeed_ = 3;
};

TEST_F(ControllerTest, speed) {
    // The speed of the returned control command should always less than the desired speed because of curvature modification.

    // Create vehicle on trajectory
    Position vehiclePosition{0, 0};
    double yaw = 0;
    Pose vehiclePose = utils::poseFromPositionandYaw(vehiclePosition, yaw);

    auto controlCommand = controller_.getControlCommand(vehiclePose, trajectory_);

    EXPECT_LE(controlCommand.speed, desiredSpeed_);
}

TEST_F(ControllerTest, velocitybound) {
    // The speed of the returned control command should always in the defined velocity bound.

    // Create vehicle on trajectory
    Position vehiclePosition{0, 0};
    double yaw = 0;
    Pose vehiclePose = utils::poseFromPositionandYaw(vehiclePosition, yaw);

    auto controlCommand = controller_.getControlCommand(vehiclePose, trajectory_);

    EXPECT_LE(controlCommand.speed, 3);
    EXPECT_GE(controlCommand.speed, 0);
}

TEST_F(ControllerTest, velocityThreshold) {
    // With th desired speed below the minVelocityThreshold the controller should return a stop command.

    // Mock a slow trajectory
    uint16_t trajectoryLength = 20;
    double xStart = -1;
    double xStep = 1;
    double desiredSpeed = 0.1;
    Trajectory slowTrajectory = mockTrajectory(trajectoryLength, xStart, xStep, desiredSpeed);

    // Create vehicle on trajectory
    Position vehiclePosition{0, 0};
    double yaw = 0;
    Pose vehiclePose = utils::poseFromPositionandYaw(vehiclePosition, yaw);

    auto controlCommand = controller_.getControlCommand(vehiclePose, slowTrajectory);

    EXPECT_DOUBLE_EQ(controlCommand.speed, 0);
}

TEST_F(ControllerTest, steerLeft) {
    // When the vehicle is to the right of the trajectory, it should steer to the left to return to the trajectory

    // Create vehicle right of the trajectory
    Position vehiclePosition{0, -1};
    double yaw = 0;
    Pose vehiclePose = utils::poseFromPositionandYaw(vehiclePosition, yaw);

    auto controlCommand = controller_.getControlCommand(vehiclePose, trajectory_);

    EXPECT_GT(controlCommand.steeringAngle, 0);
    EXPECT_LE(controlCommand.steeringAngle, parameters_.steeringAngleMax);
}

TEST_F(ControllerTest, steerLeftMax) {
    // Even when the vehicle is far away from the trajectory, the steering command should the defined limits.

    // Create vehicle far away to the right of the trajectory
    Position vehiclePosition{0, -10};
    double yaw = 0;
    Pose vehiclePose = utils::poseFromPositionandYaw(vehiclePosition, yaw);

    auto controlCommand = controller_.getControlCommand(vehiclePose, trajectory_);

    EXPECT_GT(controlCommand.steeringAngle, 0);
    EXPECT_LE(controlCommand.steeringAngle, parameters_.steeringAngleMax);
}

TEST_F(ControllerTest, steerRight) {
    // When the vehicle is to the left of the trajectory, it should steer to the right to return to the trajectory
    Position vehiclePosition{0, 1};
    double yaw = 0;
    Pose vehiclePose = utils::poseFromPositionandYaw(vehiclePosition, yaw);

    auto controlCommand = controller_.getControlCommand(vehiclePose, trajectory_);

    EXPECT_LT(controlCommand.steeringAngle, 0);
    EXPECT_GE(controlCommand.steeringAngle, -parameters_.steeringAngleMax);
}

TEST_F(ControllerTest, steerRightMax) {
    // Even when the vehicle is far away from the trajectory, the steering command should the defined limits.

    // Create vehicle far away to the left of the trajectory
    Position vehiclePosition{0, 10};
    double yaw = 0;
    Pose vehiclePose = utils::poseFromPositionandYaw(vehiclePosition, yaw);

    auto controlCommand = controller_.getControlCommand(vehiclePose, trajectory_);

    EXPECT_LT(controlCommand.steeringAngle, 0);
    EXPECT_GE(controlCommand.steeringAngle, -parameters_.steeringAngleMax);
}

TEST_F(ControllerTest, throwOnInitializedParameters) {
    // The controller should throw a runtime error if getControlCommand is called before the controller is initialized

    // Create new, uninitialized controller object
    Controller controller;

    // Create vehicle pose
    Position vehiclePosition{0, 0};
    double yaw = 0;
    Pose vehiclePose = utils::poseFromPositionandYaw(vehiclePosition, yaw);

    EXPECT_THROW(controller.getControlCommand(vehiclePose, trajectory_), std::runtime_error);
}

} // namespace kal_controller
