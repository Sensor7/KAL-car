#include "trajectory_planner.hpp"

#include <stdexcept>

#include <Eigen/Geometry>
#include <tf2_eigen/tf2_eigen.h>

#include "conversions.hpp"

namespace kal_trajectory_planner_ros_tool {

/**
 * Initialization
 */
TrajectoryPlannerNode::TrajectoryPlannerNode(const ros::NodeHandle& nhPrivate)
        : interface_{nhPrivate}, reconfigureServer_{nhPrivate} {

    interface_.fromParamServer();
    trajectoryPlanner_.setPolynomialDegree(interface_.polynomial_degree);

    /*
     * Set up callbacks for subscribers and reconfigure.
     *
     * New subscribers can be created with "add_subscriber" in "cfg/TrajectoryPlanner.if file.
     * Don't forget to register your callbacks here!
     */
    reconfigureServer_.setCallback(boost::bind(&TrajectoryPlannerNode::reconfigureCallback, this, _1, _2));
    interface_.path_subscriber->registerCallback(&TrajectoryPlannerNode::pathCallback, this);
    trajectoryGenerationTimer_ =
        nhPrivate.createTimer(ros::Rate(interface_.timer_rate), &TrajectoryPlannerNode::trajectoryCallback, this);

    interface_.showNodeInfo();

    /*
     * The preferred way of logging is to call the logging functions of the interface object.
     * These also work for nodelets and are better than using ROS_DEBUG (etc.) macros.
     */
    interface_.logDebug("Node initialized.");
}

void TrajectoryPlannerNode::pathCallback(const nav_msgs::Path::ConstPtr& pathMsg) {
    Path path = conversions::pathMsgToPath(pathMsg);
    trajectoryPlanner_.setPath(path);
    pathInitialized_ = true;
}

void TrajectoryPlannerNode::trajectoryCallback(const ros::TimerEvent& /*timer*/) {
    // Make sure path is initialized, otherwise trajectory planner will crash
    if (!pathInitialized_) {
        interface_.logWarn("Cannot compute trajectory because no path has been set yet.");
        return;
    }

    // Find vehicle pose
    Pose vehiclePose;
    if (!currentVehiclePose(vehiclePose)) {
        interface_.logWarn("Cannot compute trajectory because vehicle pose was not found in tf tree.");
        return;
    }
    // Compute trajectory
    Trajectory trajectory = trajectoryPlanner_.computeTrajectory(
        vehiclePose, interface_.desired_speed, interface_.trajectory_length, interface_.trajectory_num_of_points);

    // Publish trajectory
    interface_.trajectory_publisher.publish(conversions::trajectoryToPathMsg(trajectory, interface_.map_frame));
}


void TrajectoryPlannerNode::reconfigureCallback(const Interface::Config& config, uint32_t /*level*/) {
    interface_.fromConfig(config);
    trajectoryPlanner_.setPolynomialDegree(interface_.polynomial_degree);
}

bool TrajectoryPlannerNode::currentVehiclePose(Pose& pose) const {

    // Find vehicle position in tf tree
    geometry_msgs::TransformStamped vehiclePoseRos;
    Eigen::Isometry3d vehiclePose3d;
    try {
        vehiclePoseRos = tfBuffer_.lookupTransform(interface_.map_frame, interface_.vehicle_frame, ros::Time(0));
    } catch (const tf2::TransformException& e) {
        return false;
    }

    // Transform to Eigen type
    vehiclePose3d = tf2::transformToEigen(vehiclePoseRos);

    // Convert to two dimensions
    pose = conversions::isometry2dFromIsometry3d(vehiclePose3d);

    return true;
}


} // namespace kal_trajectory_planner_ros_tool
