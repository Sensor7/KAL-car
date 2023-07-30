// Google test docs
// wiki page: https://code.google.com/p/googletest/w/list
// primer: https://code.google.com/p/googletest/wiki/V1_7_Primer
// FAQ: https://code.google.com/p/googletest/wiki/FAQ
// advanced guide: https://code.google.com/p/googletest/wiki/V1_7_AdvancedGuide
// samples: https://code.google.com/p/googletest/wiki/V1_7_Samples

#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include "gtest/gtest.h"

#include <util_testing_ros/listener.hpp>

using util_testing::Listener;

/**
 * @brief Publish a mocked vehicle pose to the static tf tree.
 *
 * @param x
 * @param y
 * @param yaw
 */
void publishMockedVehiclePoseToTfStatic(double x, double y, double yaw) {
    static tf2_ros::StaticTransformBroadcaster staticTransformBroadcaster;
    geometry_msgs::TransformStamped vehiclePose;

    vehiclePose.header.stamp = ros::Time::now();
    vehiclePose.header.frame_id = "map";
    vehiclePose.child_frame_id = "vehicle";

    vehiclePose.transform.translation.x = x;
    vehiclePose.transform.translation.y = y;
    vehiclePose.transform.translation.z = 0.0;

    tf2::Quaternion quat;
    quat.setRPY(0, 0, yaw);
    vehiclePose.transform.rotation.x = quat.x();
    vehiclePose.transform.rotation.y = quat.y();
    vehiclePose.transform.rotation.z = quat.z();
    vehiclePose.transform.rotation.w = quat.w();

    staticTransformBroadcaster.sendTransform(vehiclePose);
}

/**
 * @brief Mock a simple path
 *
 * Path will be on equidistant points along x-axis 1m apart starting at 0m.
 *
 * @param length Amount of points the path will contain
 * @return nav_msgs::Path
 */
nav_msgs::Path mockPathMsg(size_t length) {
    nav_msgs::Path path;
    path.header.frame_id = "map";
    path.header.stamp = ros::Time(0);

    geometry_msgs::PoseStamped stampedPose;
    stampedPose.header.frame_id = "map";
    stampedPose.header.stamp = ros::Time(0);
    stampedPose.pose.position.y = 0.0;
    stampedPose.pose.position.z = 0.0;
    stampedPose.pose.orientation.w = 1.0;
    stampedPose.pose.orientation.x = 0.0;
    stampedPose.pose.orientation.y = 0.0;
    stampedPose.pose.orientation.z = 0.0;

    for (size_t i = 0; i < length; i++) {
        stampedPose.pose.position.x = static_cast<double>(i);
        path.poses.push_back((stampedPose));
    }

    return path;
}

/**
 * @brief Test that a given path msg will result in the expected trajectory msg
 *
 */
TEST(kal_trajectory_planner_ros_tool, testTrajectoryPlanerNode) {
    // Create test publisher and subscriber
    ros::NodeHandle handler;
    ros::Publisher pathAdvertiser = handler.advertise<nav_msgs::Path>("/rostest/path", 1);
    Listener<nav_msgs::Path> trajectoryListener("/rostest/trajectory", handler);

    // Publish vehicle pose to tf tree
    // Since timing is a little difficult in rostests, we will publish the vehicle pose as a static transform
    publishMockedVehiclePoseToTfStatic(0.9, 0.3, 0);

    // Wait for all ROS objects to be up and running
    auto initDelay = ros::NodeHandle("~").param("init_delay", 3.);
    ros::Duration(initDelay).sleep();

    // Ensure nodes are connected
    EXPECT_GE(trajectoryListener.subscriber.getNumPublishers(), 1);
    EXPECT_GE(pathAdvertiser.getNumSubscribers(), 1);

    // Publish path
    const nav_msgs::Path pathMsg = mockPathMsg(20);
    pathAdvertiser.publish((pathMsg));

    // Receive trajectory message
    EXPECT_TRUE(trajectoryListener.waitForMessage(ros::Duration(5), true));
    nav_msgs::Path trajectory = *trajectoryListener.msg;

    // Trajectory should have the amount of poses we defined in the parameter yaml file
    EXPECT_EQ(trajectory.poses.size(), 20);

    for (int i = 1; i < trajectory.poses.size(); i++) {
        // The speed implied by the time stamps should match the desiredSpeed
        const double distance = trajectory.poses[i].pose.position.x - trajectory.poses[i - 1].pose.position.x;
        const double deltaT = trajectory.poses[i].header.stamp.toSec() - trajectory.poses[i - 1].header.stamp.toSec();
        EXPECT_NEAR(distance / deltaT, 2.0, 1e-5);
    }

    // The first pose in the trajectory should be on the point of the path closest to the vehicle position
    EXPECT_DOUBLE_EQ(trajectory.poses.front().pose.position.x, 1.0);
}


int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "trajectory_planner_test");

    // The async spinner lets you publish and receive messages during the tests, no need to call spinOnce()
    ros::AsyncSpinner spinner(1);
    spinner.start();

    const int ret = RUN_ALL_TESTS();
    ros::shutdown();
    return ret;
}
