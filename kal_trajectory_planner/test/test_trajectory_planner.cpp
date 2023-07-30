// google test docs
// wiki page: https://code.google.com/p/googletest/w/list
// primer: https://code.google.com/p/googletest/wiki/V1_7_Primer
// FAQ: https://code.google.com/p/googletest/wiki/FAQ
// advanced guide: https://code.google.com/p/googletest/wiki/V1_7_AdvancedGuide
// samples: https://code.google.com/p/googletest/wiki/V1_7_Samples
//
#include "kal_trajectory_planner/trajectory_planner.hpp"

#include <cstdint>
#include <stdexcept>

#include <glog/logging.h>
#include "gtest/gtest.h"

#include "kal_trajectory_planner/types.hpp"

namespace kal_trajectory_planner {


// The setup function will be called before the execution of each unit test. This saves some copy & paste work.
// Make sure to declare unit test as TEST_F and you will be able to access the protected variables of this class
class TrajectoryPlannerTest : public testing::Test {
protected:
    void SetUp() override {
        // Mock path
        for (int i = 0; i < 10; i++) {
            path_.push_back(Position(i, 0));
        }
    }

    Path path_;

    double desiredSpeed_ = 2;
    double trajectoryLength_ = 4;
    uint64_t numOfPoints_ = 20;
};


TEST_F(TrajectoryPlannerTest, computeTrajectory) {
    TrajectoryPlanner trajectoryPlanner;

    // Create vehicle pose near the beginning of the path
    Eigen::Translation2d translation(1.1, 0.5);
    Eigen::Rotation2Dd rotation(0);
    Pose vehiclePose = translation * rotation;

    // This shouldn't work because we have not yet set the path
    EXPECT_THROW(trajectoryPlanner.computeTrajectory(vehiclePose, desiredSpeed_, trajectoryLength_, numOfPoints_),
                 std::runtime_error);

    // Compute trajectory
    trajectoryPlanner.setPath(path_);
    Trajectory trajectory =
        trajectoryPlanner.computeTrajectory(vehiclePose, desiredSpeed_, trajectoryLength_, numOfPoints_);

    // The computed trajectory should have the previously set number of points, no matter what
    EXPECT_EQ(trajectory.size(), numOfPoints_);

    double x = 1;
    double stepWidth = trajectoryLength_ / static_cast<double>(numOfPoints_);
    for (int i = 0; i < numOfPoints_; i++) {
        // Computed positions should match expectations
        EXPECT_DOUBLE_EQ(trajectory[i].position.x(), x);

        x += stepWidth;
    }
    for (int i = 1; i < numOfPoints_; i++) {
        // The speed implied by the time stamps should match the desiredSpeed
        const double distance = (trajectory[i].position - trajectory[i - 1].position).norm();
        const double deltaT =
            trajectory[i].stamp.time_since_epoch().count() - trajectory[i - 1].stamp.time_since_epoch().count();
        EXPECT_NEAR(distance / deltaT, desiredSpeed_, 1e-7);
    }
}


TEST_F(TrajectoryPlannerTest, computeTrajectoryCloseToEndOfPath) {
    TrajectoryPlanner trajectoryPlanner;

    // Create vehicle pose near the end of the path
    Eigen::Translation2d translation(7.1, -0.5);
    Eigen::Rotation2Dd rotation(0);
    Pose vehiclePose = translation * rotation;

    // Compute trajectory
    trajectoryPlanner.setPath(path_);
    Trajectory trajectory =
        trajectoryPlanner.computeTrajectory(vehiclePose, desiredSpeed_, trajectoryLength_, numOfPoints_);

    // The computed trajectory should have the previously set number of points, no matter what
    EXPECT_EQ(trajectory.size(), numOfPoints_);

    for (int i = 1; i < numOfPoints_; i++) {

        // We should not plan past the end of the path
        EXPECT_LT(trajectory[i].position.x(), 9);
        ;
    }
    for (int i = 1; i < numOfPoints_; i++) {
        // Starting close to the end, the speed should decrease
        const double distance = (trajectory[i].position - trajectory[i - 1].position).norm();
        const double deltaT =
            trajectory[i].stamp.time_since_epoch().count() - trajectory[i - 1].stamp.time_since_epoch().count();
        EXPECT_LE(distance / deltaT, desiredSpeed_);
    }
}

TEST_F(TrajectoryPlannerTest, computeTrajectoryEndOfPath) {
    TrajectoryPlanner trajectoryPlanner;

    // Create vehicle pose behind the end of the path
    Eigen::Translation2d translation(100, -25);
    Eigen::Rotation2Dd rotation(0);
    Pose vehiclePose = translation * rotation;

    // Compute trajectory
    trajectoryPlanner.setPath(path_);
    Trajectory trajectory =
        trajectoryPlanner.computeTrajectory(vehiclePose, desiredSpeed_, trajectoryLength_, numOfPoints_);

    // The computed trajectory should have the previously set number of points, no matter what
    EXPECT_EQ(trajectory.size(), numOfPoints_);

    for (int i = 0; i < numOfPoints_; i++) {

        // All computed positions shoudl match the last point on the path since we started behind the end
        EXPECT_DOUBLE_EQ(trajectory[i].position.x(), 9);
    }
    for (int i = 1; i < numOfPoints_; i++) {
        // Starting at the end, all returned poses will be identical with the time step advanced by 1s from pose to pose
        const double distance = (trajectory[i].position - trajectory[i - 1].position).norm();
        const double deltaT =
            trajectory[i].stamp.time_since_epoch().count() - trajectory[i - 1].stamp.time_since_epoch().count();
        EXPECT_DOUBLE_EQ(distance, 0);
        EXPECT_DOUBLE_EQ(deltaT, 1);
    }
}


int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = google::INFO;
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

} // namespace kal_trajectory_planner