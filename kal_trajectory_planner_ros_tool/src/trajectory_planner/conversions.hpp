#pragma once

#include <chrono>
#include <stdexcept>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <kal_trajectory_planner/types.hpp>

namespace kal_trajectory_planner_ros_tool::conversions {
using namespace kal_trajectory_planner;

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
 * @brief Transform a nav_msgs::Path into a Path type
 *
 * @param pathMsg
 * @return Path
 */
inline Path pathMsgToPath(const nav_msgs::Path::ConstPtr& pathMsg) {
    Path path;
    for (auto const& poseStamped : pathMsg->poses) {
        path.push_back(Position(poseStamped.pose.position.x, poseStamped.pose.position.y));
    }

    return path;
}

/**
 * @brief Transform the Trajctory type into a nav_msgs::Path
 *
 * @param trajectory
 * @param frameId
 * @return nav_msgs::Path
 */
inline nav_msgs::Path trajectoryToPathMsg(const Trajectory& trajectory, const std::string& frameId) {
    nav_msgs::Path pathMsg;
    pathMsg.header.frame_id = frameId;
    pathMsg.header.stamp = toRosTime(trajectory.front().stamp);

    geometry_msgs::PoseStamped stampedPoseRos;
    stampedPoseRos.header.frame_id = frameId;
    stampedPoseRos.pose.orientation.w = 1;
    stampedPoseRos.pose.orientation.x = 0;
    stampedPoseRos.pose.orientation.y = 0;
    stampedPoseRos.pose.orientation.z = 0;

    for (auto const& stampedPosition : trajectory) {
        stampedPoseRos.header.stamp = toRosTime(stampedPosition.stamp);
        stampedPoseRos.pose.position.x = stampedPosition.position.x();
        stampedPoseRos.pose.position.y = stampedPosition.position.y();

        pathMsg.poses.push_back(stampedPoseRos);
    }

    return pathMsg;
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

} // namespace kal_trajectory_planner_ros_tool::conversions