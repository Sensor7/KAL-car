// google test docs
// wiki page: https://code.google.com/p/googletest/w/list
// primer: https://code.google.com/p/googletest/wiki/V1_7_Primer
// FAQ: https://code.google.com/p/googletest/wiki/FAQ
// advanced guide: https://code.google.com/p/googletest/wiki/V1_7_AdvancedGuide
// samples: https://code.google.com/p/googletest/wiki/V1_7_Samples
//
#include <chrono>
#include <tuple>
#include "kal3_controller/internal/utils.hpp"

#include <Eigen/Geometry>
#include <glog/logging.h>
#include "gtest/gtest.h"

#include "kal3_controller/types.hpp"

namespace kal3_controller::utils {

TEST(UtilsTest, findIndexOfClosestPointOnTrajectory) {
    // Mock trajectory
    Trajectory trajectory;
    for (int i = 0; i < 3; i++) {
        StampedPose stampedPose;
        stampedPose.stamp = Time::clock::now();
        stampedPose.pose = Eigen::Translation2d(i, 0) * Eigen::Rotation2Dd(0);
        trajectory.push_back(stampedPose);
    }

    // Position close to path
    Position positionCloseToPath(1, 0.5);
    auto indexOfClostestPoint = findIndexOfClosestPointOnTrajectory(trajectory, positionCloseToPath);
    EXPECT_EQ(indexOfClostestPoint, 1);


    // Position in front of path
    Position positionInFrontOfPath(-100, 20);
    indexOfClostestPoint = findIndexOfClosestPointOnTrajectory(trajectory, positionInFrontOfPath);
    EXPECT_EQ(indexOfClostestPoint, 0);
}

TEST(UtilsTest, discreteCurvature) {
    // Create three points on upper half of circle
    double radius = 2;
    Eigen::Vector2d pointA{-radius, 0};
    Eigen::Vector2d pointB{0, radius};
    Eigen::Vector2d pointC{radius, 0};

    double curvatureUpperCircle = discreteCurvature(pointA, pointB, pointC);
    EXPECT_DOUBLE_EQ(curvatureUpperCircle, -1 / radius);

    // Create point on lower half of circle
    Eigen::Vector2d pointD{0, -radius};
    double curvatureLowerCircle = discreteCurvature(pointA, pointD, pointC);
    EXPECT_DOUBLE_EQ(curvatureLowerCircle, 1 / radius);
}

TEST(UtilsTest, computeDesiredSpeed) {
    Position positionA{5, 0};
    Position positionB{10, 0};
    double yaw = 0;

    double distanceTravelled = (positionB - positionA).norm();
    double speed = 2.5;
    Time now = std::chrono::steady_clock::now();
    Duration duration = std::chrono::duration_cast<Duration>(std::chrono::duration<double>(distanceTravelled / speed));
    Time stamp(now + duration);

    Pose poseA = poseFromPositionandYaw(positionA, yaw);
    Pose poseB = poseFromPositionandYaw(positionB, yaw);

    StampedPose first{poseA, now};
    StampedPose second{poseB, stamp};

    EXPECT_DOUBLE_EQ(computeDesiredSpeed(first, second), speed);
}

TEST(UtilsTest, signedDistanceBetweenPointAndLine) {
    // Define line along y axis
    Eigen::Vector2d linePoint1{0, -1};
    Eigen::Vector2d linePoint2{0, 0};

    // Define point left of the line
    Eigen::Vector2d pointLeftOfCurve{-1, 1};
    double distanceLeftOfCurve =
        signedDistanceBetweenPointAndLine(pointLeftOfCurve, std::make_tuple(linePoint1, linePoint2));
    EXPECT_DOUBLE_EQ(distanceLeftOfCurve, 1);

    // Define point right of the line
    Eigen::Vector2d pointRightOfCurve{15, -5};
    double distanceRightOfCurve =
        signedDistanceBetweenPointAndLine(pointRightOfCurve, std::make_tuple(linePoint1, linePoint2));
    EXPECT_DOUBLE_EQ(distanceRightOfCurve, -15);
}


TEST(UtilsTest, normalizeAnglePlusMinusPi) {
    EXPECT_DOUBLE_EQ(normalizeAnglePlusMinusPi(0), 0);
    EXPECT_DOUBLE_EQ(normalizeAnglePlusMinusPi(-M_PI), -M_PI);
    EXPECT_DOUBLE_EQ(normalizeAnglePlusMinusPi(M_PI), M_PI);
    EXPECT_NEAR(normalizeAnglePlusMinusPi(2 * M_PI), 0, 1e-15);
    EXPECT_DOUBLE_EQ(normalizeAnglePlusMinusPi(5 * M_PI_2), M_PI_2);
}

} // namespace kal_controller::utils