#pragma once

#include <chrono>
#include <stdexcept>

#include <Eigen/Geometry>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <ros/time.h>
#include <std_msgs/Float64.h>
#include <visualization_msgs/Marker.h>

#include <kal3_controller/types.hpp>

namespace kal3_controller_ros_tool::conversions {
using namespace kal3_controller;

/**
 * @brief Convert a std::chrono time to ros::Time
 *
 * @tparam TimeT
 * @param from
 * @return ros::Time
 */
template <typename TimeT>
inline ros::Time toRosTime(const TimeT& from) {
    ros::Time t;
    const auto nSec = std::chrono::duration_cast<std::chrono::nanoseconds>(from.time_since_epoch()).count();
    if (nSec < 0) {
        throw std::runtime_error("Invalid time given: " + std::to_string(nSec));
    }
    t.fromNSec(nSec);
    return t;
}


/**
 * @brief Convert a ros::Time to std::chrono clock
 *
 */
template <typename TimeT = std::chrono::steady_clock::time_point>
inline TimeT toChrono(const ros::Time& time) {
    return TimeT(std::chrono::nanoseconds(time.toNSec()));
}

/**
 * @brief Convert a geometry_msgs::Pose into an Eigen::Isometry3d
 *
 * @param msg
 * @return Eigen::Isometry3d
 */
inline Eigen::Isometry3d poseMsgToIsometry3d(const geometry_msgs::Pose& msg) {
    return Eigen::Isometry3d(
        Eigen::Translation3d(msg.position.x, msg.position.y, msg.position.z) *
        Eigen::Quaterniond(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z));
}

/**
 * @brief Create Eigen::Isometry2d from x and y translation and rotation around z axis of Eigen::Isometry3d
 *
 * @param isometry3d
 * @return Eigen::Isometry2d
 */
inline Eigen::Isometry2d isometry2dFromIsometry3d(const Eigen::Isometry3d& isometry3d) {
    Eigen::Translation2d translation(isometry3d.translation().head(2));
    Eigen::Rotation2Dd rotation(isometry3d.rotation().topLeftCorner<2, 2>());

    return translation * rotation;
}

/**
 * @brief Transform a nav_msgs::Trajectory into a Trajectory type
 *
 * @param TrajectoryMsg
 * @return Trajectory
 */
inline Trajectory trajectoryMsgToTrajectory(const nav_msgs::Path::ConstPtr& trajectoryMsg) {
    Trajectory trajectory;
    for (auto const& stampedPoseMsg : trajectoryMsg->poses) {
        Eigen::Isometry3d vehiclePose3d = poseMsgToIsometry3d(stampedPoseMsg.pose);
        Pose vehiclePose2d = isometry2dFromIsometry3d(vehiclePose3d);
        StampedPose stampedPose{vehiclePose2d, toChrono(stampedPoseMsg.header.stamp)};
        trajectory.push_back(stampedPose);
    }

    return trajectory;
}

/**
 * @brief Convert controlCommand to AckermanDriveStamped
 *
 * @param controlCommand
 * @param stamp
 * @param frameId
 * @return ackermann_msgs::AckermannDriveStamped
 */
inline ackermann_msgs::AckermannDriveStamped controlCommandToAckermannDriveStamped(const ControlCommand& controlCommand,
                                                                                   const ros::Time& stamp,
                                                                                   const std::string& frameId) {
    ackermann_msgs::AckermannDriveStamped msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = frameId;
    msg.drive.speed = static_cast<float>(controlCommand.speed);
    msg.drive.steering_angle = static_cast<float>(controlCommand.steeringAngle);

    return msg;
}

/**
 * @brief Convert an Eigen::Vector2d into a geometry_msgs::PointStamped
 *
 * @param vector
 * @param stamp
 * @return geometry_msgs::PointStamped
 */
inline geometry_msgs::PointStamped eigenVector2dToPointStampedMsg(const Eigen::Vector2d& vector,
                                                                  const ros::Time& stamp,
                                                                  const std::string& frameId) {
    geometry_msgs::PointStamped msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = frameId;
    msg.point.x = vector.x();
    msg.point.y = vector.y();
    msg.point.z = 0.;
    return msg;
}

/**
 * @brief Convert a double to a std_msgs::Float64
 *
 * @param value
 * @return std_msgs::Float64
 */
inline std_msgs::Float64 doubleToFloat64Msg(const double value) {
    std_msgs::Float64 msg;
    msg.data = value;
    return msg;
}

visualization_msgs::Marker controlCommandToMarkerMsg(const ControlCommand& controlCommand,
                                                     const ros::Time& stamp,
                                                     const std::string& frameId,
                                                     const double& scale = 0.1) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = frameId;
    marker.header.stamp = stamp;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = scale * controlCommand.speed;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;

    marker.pose.position.x = 0.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.0;

    Eigen::Quaterniond quaternion;
    quaternion = Eigen::AngleAxisd(controlCommand.steeringAngle, Eigen::Vector3d::UnitZ());
    marker.pose.orientation.x = quaternion.x();
    marker.pose.orientation.y = quaternion.y();
    marker.pose.orientation.z = quaternion.z();
    marker.pose.orientation.w = quaternion.w();

    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    return marker;
}

} // namespace kal_controller_ros_tool::conversions
