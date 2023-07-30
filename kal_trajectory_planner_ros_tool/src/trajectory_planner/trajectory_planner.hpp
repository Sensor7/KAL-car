#pragma once

#include <dynamic_reconfigure/server.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>

#include <kal_trajectory_planner/trajectory_planner.hpp>
#include <kal_trajectory_planner/types.hpp>

#include "kal_trajectory_planner_ros_tool/TrajectoryPlannerInterface.h"
namespace kal_trajectory_planner_ros_tool {

using namespace kal_trajectory_planner;

class TrajectoryPlannerNode {
public:
    using Interface = TrajectoryPlannerInterface;

    explicit TrajectoryPlannerNode(const ros::NodeHandle& nhPrivate);

private:
    /**
     * @brief Callback to pass a new path to the trajectory planner.
     *
     * Path will be filtered to contain only points with a minimal euclidean distance.
     *
     * @param pathMsg
     */
    void pathCallback(const nav_msgs::Path::ConstPtr& pathMsg);

    /**
     * @brief Timed callback that will compute and publish a new trajectory.
     *
     */
    void trajectoryCallback(const ros::TimerEvent&);

    /**
     * @brief Dynamic reconfigure callback to update parameter values.
     *
     * This callback is called at startup or whenever a change was made in the dynamic_reconfigure window.
     * The parameter "level" is unused here. It is set to the number of changes in the config.
     * At startup, the callback is automatically called with level == std::numeric_limits<uint32_t>::max().
     */
    void reconfigureCallback(const Interface::Config& config, uint32_t /*level*/);

    /**
     * @brief Read current vehicle pose from tf tree
     *
     * @param pose Return parameter. Contains the pose, if found.
     * @return true Pose found successfully
     * @return false Pose not found
     */
    bool currentVehiclePose(Pose& pose) const;

    TrajectoryPlanner trajectoryPlanner_;

    ros::Timer trajectoryGenerationTimer_;
    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener tfListener_{tfBuffer_};

    Interface interface_;
    dynamic_reconfigure::Server<Interface::Config> reconfigureServer_;

    bool pathInitialized_{false};
};
} // namespace kal_trajectory_planner_ros_tool
