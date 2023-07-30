// Google test docs
// wiki page: https://code.google.com/p/googletest/w/list
// primer: https://code.google.com/p/googletest/wiki/V1_7_Primer
// FAQ: https://code.google.com/p/googletest/wiki/FAQ
// advanced guide: https://code.google.com/p/googletest/wiki/V1_7_AdvancedGuide
// samples: https://code.google.com/p/googletest/wiki/V1_7_Samples

#include <ackermann_msgs/AckermannDriveStamped.h>
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
 * @brief Mock a simple trajectory
 *
 * Trajectory will be on equidistant points along x-axis 1m and 1s apart starting at 0m with 0deg yaw.
 *
 * @param length Amount of points the trajectory will contain
 * @return nav_msgs::Path
 */
nav_msgs::Path mockTrajectoryMsg(size_t length) {
    nav_msgs::Path trajectory;
    trajectory.header.frame_id = "map";
    trajectory.header.stamp = ros::Time(0);

    geometry_msgs::PoseStamped stampedPose;
    stampedPose.header.frame_id = "map";
    stampedPose.pose.position.y = 0.0;
    stampedPose.pose.position.z = 0.0;
    stampedPose.pose.orientation.w = 1.0;
    stampedPose.pose.orientation.x = 0.0;
    stampedPose.pose.orientation.y = 0.0;
    stampedPose.pose.orientation.z = 0.0;

    for (size_t i = 0; i < length; i++) {
        stampedPose.pose.position.x = static_cast<double>(i);
        stampedPose.header.stamp = ros::Time(static_cast<double>(i));
        trajectory.poses.push_back((stampedPose));
    }

    return trajectory;
}

/**
 * @brief Test that a given trajectory msg will result in the expected control command msg
 *
 */
TEST(kal3_controller_ros_tool, testControllerNode) {
    // Create test publisher and subscriber
    ros::NodeHandle handler;
    ros::Publisher trajectoryAdvertiser = handler.advertise<nav_msgs::Path>("/rostest/trajectory", 1);
    Listener<ackermann_msgs::AckermannDriveStamped> controlCommandListener("/rostest/control_command", handler);

    // Publish vehicle pose to tf tree
    // Since timing is a little difficult in rostests, we will publish the vehicle pose as a static transform
    publishMockedVehiclePoseToTfStatic(0.9, -0.8, 0);

    // Wait for all ROS objects to be up and running
    auto initDelay = ros::NodeHandle("~").param("init_delay", 3.);
    ros::Duration(initDelay).sleep();

    // Ensure nodes are connected
    EXPECT_GE(controlCommandListener.subscriber.getNumPublishers(), 1);
    EXPECT_GE(trajectoryAdvertiser.getNumSubscribers(), 1);

    // Publish trajectory
    const nav_msgs::Path trajectoryMsg = mockTrajectoryMsg(20);
    trajectoryAdvertiser.publish((trajectoryMsg));

    // Receive control command message
    EXPECT_TRUE(controlCommandListener.waitForMessage(ros::Duration(5), true));
    ackermann_msgs::AckermannDriveStamped controlCommand = *controlCommandListener.msg;

    // The trajectory time stamps in distances imply a velocity less than 1m/s
    EXPECT_LE(controlCommand.drive.speed, 1);

    // The vehicle is on the right hand side of the trajectory, expect a positive steering angle
    EXPECT_GT(controlCommand.drive.steering_angle, 0);

    // The steering angle should never be greater than the defined maximum
    EXPECT_LE(controlCommand.drive.steering_angle, 0.31);
}

/**
 * @brief Test that not setting the trajectory will result in a stop command
 *
 */
TEST(kal3_controller_ros_tool, testControllerNodeNoTrajectory) {
    // Create test publisher and subscriber
    ros::NodeHandle handler;
    Listener<ackermann_msgs::AckermannDriveStamped> controlCommandListener("/rostest/control_command", handler);

    // Publish vehicle pose to tf tree
    // Since timing is a little difficult in rostests, we will publish the vehicle pose as a static transform
    publishMockedVehiclePoseToTfStatic(-0.9, 0.3, 0);

    // Wait for all ROS objects to be up and running
    auto initDelay = ros::NodeHandle("~").param("init_delay", 3.);
    ros::Duration(initDelay).sleep();

    // Receive control command message
    EXPECT_TRUE(controlCommandListener.waitForMessage(ros::Duration(5), true));
    ackermann_msgs::AckermannDriveStamped controlCommand = *controlCommandListener.msg;

    // We did not set a trajectory so we expect the vehicle to stop
    EXPECT_DOUBLE_EQ(controlCommand.drive.speed, 0);
    EXPECT_DOUBLE_EQ(controlCommand.drive.steering_angle, 0);
}

/**
 * @brief Test the vehicle will stop if the trajectory is too old
 *
 */
TEST(kal3_controller_ros_tool, testControllerNodeOldTrajectory) {
    // Create test publisher and subscriber
    ros::NodeHandle handler;
    ros::Publisher trajectoryAdvertiser = handler.advertise<nav_msgs::Path>("/rostest/trajectory", 1);
    Listener<ackermann_msgs::AckermannDriveStamped> controlCommandListener("/rostest/control_command", handler);

    // Publish vehicle pose to tf tree
    // Since timing is a little difficult in rostests, we will publish the vehicle pose as a static transform
    publishMockedVehiclePoseToTfStatic(-0.9, 0.3, 0);

    // Wait for all ROS objects to be up and running
    auto initDelay = ros::NodeHandle("~").param("init_delay", 3.);
    ros::Duration(initDelay).sleep();

    // Publish trajectory
    const nav_msgs::Path trajectoryMsg = mockTrajectoryMsg(20);
    trajectoryAdvertiser.publish((trajectoryMsg));

    // Sleep longer than the maximum allowed trajectory age
    ros::Duration(1.5).sleep();

    // Receive control command message
    EXPECT_TRUE(controlCommandListener.waitForMessage(ros::Duration(5), true));
    ackermann_msgs::AckermannDriveStamped controlCommand = *controlCommandListener.msg;

    // Since the trajectory is too old by now, the vehicle should stop
    EXPECT_DOUBLE_EQ(controlCommand.drive.speed, 0);
    EXPECT_DOUBLE_EQ(controlCommand.drive.steering_angle, 0);
}

/**
 * @brief Test that a given trajectory that's too short will result in the vehicle stopping
 *
 */
TEST(kal3_controller_ros_tool, testControllerNodeShortTrajectory) {
    // Create test publisher and subscriber
    ros::NodeHandle handler;
    ros::Publisher trajectoryAdvertiser = handler.advertise<nav_msgs::Path>("/rostest/trajectory", 1);
    Listener<ackermann_msgs::AckermannDriveStamped> controlCommandListener("/rostest/control_command", handler);

    // Publish vehicle pose to tf tree
    // Since timing is a little difficult in rostests, we will publish the vehicle pose as a static transform
    publishMockedVehiclePoseToTfStatic(0.9, -0.8, 0);

    // Wait for all ROS objects to be up and running
    auto initDelay = ros::NodeHandle("~").param("init_delay", 3.);
    ros::Duration(initDelay).sleep();

    // Publish trajectory
    const nav_msgs::Path trajectoryMsg = mockTrajectoryMsg(5);
    trajectoryAdvertiser.publish((trajectoryMsg));

    // Receive control command message
    EXPECT_TRUE(controlCommandListener.waitForMessage(ros::Duration(5), true));
    ackermann_msgs::AckermannDriveStamped controlCommand = *controlCommandListener.msg;

    // Since the trajectory is too short, the vehicle should stop
    EXPECT_DOUBLE_EQ(controlCommand.drive.speed, 0);
    EXPECT_DOUBLE_EQ(controlCommand.drive.steering_angle, 0);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "controller_test");

    // The async spinner lets you publish and receive messages during the tests, no need to call spinOnce()
    ros::AsyncSpinner spinner(1);
    spinner.start();

    int ret = RUN_ALL_TESTS();
    ros::shutdown();
    return ret;
}
