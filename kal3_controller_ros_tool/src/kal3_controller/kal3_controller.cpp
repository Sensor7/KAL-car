#include "kal3_controller.hpp"

#include <ackermann_msgs/AckermannDriveStamped.h>
#include <ros/time.h>
#include <tf2_eigen/tf2_eigen.h>
#include <visualization_msgs/Marker.h>

#include "conversions.hpp"


namespace kal3_controller_ros_tool {

/**
 * Initialization
 */
ControllerNode::ControllerNode(const ros::NodeHandle& nhPrivate)
        : interface_{nhPrivate}, reconfigureServer_{nhPrivate} {

    interface_.fromParamServer();
    setControllerParameters();
    /*
     * Set up callbacks for subscribers and reconfigure.
     *
     * New subscribers can be created with "add_subscriber" in "cfg/Controller.if file.
     * Don't forget to register your callbacks here!
     */
    reconfigureServer_.setCallback(boost::bind(&ControllerNode::reconfigureCallback, this, _1, _2));
    interface_.trajectory_subscriber->registerCallback(&ControllerNode::trajectoryCallback, this);
    interface_.cone_trajectory_subscriber->registerCallback(&ControllerNode::trajectoryconesCallback, this);
    interface_.stop_sign_subscriber->registerCallback(&ControllerNode::stopsigndisCallback, this);
    interface_.cone_dist_subscriber->registerCallback(&ControllerNode::conedisCallback, this);
    controlLoopTimer_ =
        nhPrivate.createTimer(ros::Rate(interface_.control_loop_rate), &ControllerNode::controlLoopCallback, this);

    interface_.showNodeInfo();

    /*
     * The preferred way of logging is to call the logging functions of the interface object.
     * These also work for nodelets and are better than using ROS_DEBUG (etc.) macros.
     */
    interface_.logDebug("Node initialized.");
}


void ControllerNode::trajectoryCallback(const nav_msgs::Path::ConstPtr& trajectoryMsg) {
    trajectoryStamp_ = ros::Time::now();
    trajectory_ = conversions::trajectoryMsgToTrajectory(trajectoryMsg);
}

void ControllerNode::trajectoryconesCallback(const nav_msgs::Path::ConstPtr& trajectoryMsg_c) {
    trajectoryStamp_cone = ros::Time::now();
    trajectory_cone = conversions::trajectoryMsgToTrajectory(trajectoryMsg_c);
    interface_.logWarn("New trajectory in cones detected. ");
}

void ControllerNode::stopsigndisCallback(const std_msgs::Float64& sign_dist) {
    sign_dist_ = sign_dist.data;
}

void ControllerNode::conedisCallback(const std_msgs::Float64& cone_dist) {
    cone_dist_ = cone_dist.data;
}

void ControllerNode::controlLoopCallback(const ros::TimerEvent& timerEvent) {
    if (!checkTrajectoryExists()) {
        return;
    }
    if (!checkTrajectoryAge()) {
        return;
    }
    if (!checkTrajectoryLength()) {
        return;
    }

    // Find vehicle pose
    Pose vehiclePose;
    if (!currentVehiclePose(vehiclePose)) {
        return;
    }

    //Counting the race loop
    Position endpoint(4.8,0.7);
    if ((vehiclePose.translation() - endpoint).norm() < 0.8) {
        double running_time = (ros::Time::now() - time_global_).toSec();
        if (running_time > 10 ) {
            circle_num = circle_num + 1 ;
            time_global_ = ros::Time::now();
        }

        if (circle_num == 3)  {
            circle_num = 0;
        }
        // std::stringstream outStream;
        // outStream << std::fixed << std::setprecision(0) << "Now is at the " << circle_num
        //           << " loop. ";
        // interface_.logWarn(outStream.str());
    }

    std::stringstream outStream;
    outStream << std::fixed << std::setprecision(0) << "Now is at the " << circle_num
                  << " loop. ";
    interface_.logWarn(outStream.str());

    //detect distance to stop sign
    if (!checkStopsignDistance()) {
        ros::Duration du(3);
        controller_.init_command_record();
        controller_.reset_control_deque();
        du.sleep();
        stop_flag_ = true;
        return;
    }

    checkConeDistance();
    ControlCommand controlCommand;

    //stop the car if no trajectory updated in 1s
    // if (((ros::Time::now() - trajectoryStamp_cone).toSec()) > 2) {
    //     publishStopCommand();
    //     interface_.logWarn("No trajectory updated for 1s. Stop the car. ");
    //     cone_flag_ = false;
    //     return;
    // }

    //controll command for testing path through the cones
    // if (trajectory_cone && cone_flag_ ) {
    //     controlCommand =
    //     controller_.getControlCommand(vehiclePose, *trajectory_cone, 0.2, true, interface_.publish_debug_info);
    // }
    // else {
    //     publishStopCommand();
    //     interface_.logWarn("Conditions for trajectory not satisfied. Stop the car. ");
    //     return;
    // }

    //using the trajectory through the cones if conditions satisfied
    if (trajectory_cone && cone_flag_ && circle_num == 2) {
        controlCommand =
        controller_.getControlCommand(vehiclePose, *trajectory_cone, 0.2, true, interface_.publish_debug_info);
        interface_.logWarn("Switch to the new trajectory!!! ");

        if (((ros::Time::now() - trajectoryStamp_cone).toSec()) > 1) {
            publishStopCommand();
            interface_.logWarn("No trajectory updated for 1s. Stop the car. ");
            cone_flag_ = false;
            return;
        }
    }

    //otherwise use the trajectory from demo trajectory generator
    else {
    desirespeed = 1.5;
    desirespeed = std::clamp(desirespeed, 0.0, 3.0);
    controlCommand =
        controller_.getControlCommand(vehiclePose, *trajectory_, desirespeed, false, interface_.publish_debug_info);
    }

    controlCommand.speed = std::clamp(controlCommand.speed, 0.0, 3.0);
    ros::Time stamp = ros::Time::now();
    auto controlCommandMsg =
        conversions::controlCommandToAckermannDriveStamped(controlCommand, stamp, interface_.vehicle_frame);
    interface_.control_command_publisher.publish(controlCommandMsg);
    visualization_msgs::Marker debugControlCommandViz =
        conversions::controlCommandToMarkerMsg(controlCommand, stamp, interface_.vehicle_frame);
    interface_.debug_control_command_viz_publisher.publish(debugControlCommandViz);

    if (controlCommand.debugInfo) {
        publishDebugInfo(controlCommand.debugInfo.value(), stamp);
    }
}

void ControllerNode::reconfigureCallback(const Interface::Config& config, uint32_t /*level*/) {
    interface_.fromConfig(config);
    setControllerParameters();
}

bool ControllerNode::currentVehiclePose(Pose& pose) const {
    // Find vehicle position in tf tree
    geometry_msgs::TransformStamped vehiclePoseRos;
    Eigen::Isometry3d vehiclePose3d;
    try {
        vehiclePoseRos = tfBuffer_.lookupTransform(interface_.map_frame, interface_.vehicle_frame, ros::Time(0));
    } catch (const tf2::TransformException& e) {
        interface_.logWarn("Cannot find vehicle pose. Stopping the vehicle.");
        publishStopCommand();
        return false;
    }

    // Transform to Eigen type
    vehiclePose3d = tf2::transformToEigen(vehiclePoseRos);

    // Convert to two dimensions
    pose = conversions::isometry2dFromIsometry3d(vehiclePose3d);

    return true;
}

bool ControllerNode::checkTrajectoryExists() const {
    if (!trajectory_) {
        interface_.logWarn("Trajectory undefined. Stopping the vehicle.");
        publishStopCommand();
        return false;
    }

    return true;
}

bool ControllerNode::checkTrajectoryAge() const {
    double trajectoryAge = (ros::Time::now() - trajectoryStamp_).toSec();
    if (trajectoryAge > interface_.trajectory_age_max) {
        std::stringstream outStream;
        outStream << std::fixed << std::setprecision(2) << "Trajectory is " << trajectoryAge << "s old. Limit is "
                  << interface_.trajectory_age_max << "s. Stopping the vehicle.";
        interface_.logWarn(outStream.str());
        publishStopCommand();
        return false;
    }

    return true;
}

bool ControllerNode::checkTrajectoryLength() const {
    uint16_t trajectoryMinimumLength = interface_.look_ahead_index + 1;
    if (trajectory_->size() < trajectoryMinimumLength) {
        std::stringstream outStream;
        outStream << std::fixed << std::setprecision(0) << "Trajectory only contains " << trajectory_->size()
                  << " poses. Required minimum is " << trajectoryMinimumLength << ". Stopping the vehicle.";
        interface_.logWarn(outStream.str());
        publishStopCommand();
        return false;
    }

    return true;
}

bool ControllerNode::checkStopsignDistance() {
    if (sign_dist_ < 1.5 && stop_flag_ == false) {
        std::stringstream outStream;
        outStream << std::fixed << std::setprecision(2) << "Detected stop sign in " << sign_dist_
                  << " m. Stopping the vehicle. ";
        interface_.logWarn(outStream.str());
        publishStopCommand();
        time_stop_ = ros::Time::now();
        return false;
    }

    if (((ros::Time::now() - time_stop_).toSec()) > 10) {
        stop_flag_ = false;
    }

    return true;
}

void ControllerNode::checkConeDistance() {
    if (cone_dist_ < 0.7) {
        std::stringstream outStream;
        outStream << std::fixed << std::setprecision(2) << "Detected the nearest cone in " << cone_dist_
                  << " m. ";
        interface_.logWarn(outStream.str());
        cone_flag_ = true;
        time_cone_ = ros::Time::now();
    }

    if (((ros::Time::now() - time_cone_ ).toSec()) > 2) {
        cone_flag_ = false;
    }
}

void ControllerNode::publishStopCommand() const {
    ackermann_msgs::AckermannDriveStamped stopCommandMsg;
    stopCommandMsg.header.stamp = ros::Time::now();
    stopCommandMsg.header.frame_id = interface_.vehicle_frame;
    stopCommandMsg.drive.speed = 0;
    stopCommandMsg.drive.steering_angle = 0;
    interface_.control_command_publisher.publish(stopCommandMsg);
}

void ControllerNode::publishDebugInfo(const ControlCommand::DebugInfo& debugInfo, const ros::Time& stamp) const {
    interface_.debug_closest_point_on_trajectory_publisher.publish(
        conversions::eigenVector2dToPointStampedMsg(debugInfo.closestPointOnTrajectory, stamp, interface_.map_frame));
    interface_.debug_look_ahead_point_publisher.publish(
        conversions::eigenVector2dToPointStampedMsg(debugInfo.lookAheadPoint, stamp, interface_.map_frame));
    interface_.debug_angle_path.publish(conversions::doubleToFloat64Msg(debugInfo.anglePath));
    interface_.debug_curvature.publish(conversions::doubleToFloat64Msg(debugInfo.curvature));
    interface_.debug_error_angle.publish(conversions::doubleToFloat64Msg(debugInfo.errorAngle));
    interface_.debug_error_signed_distance.publish(conversions::doubleToFloat64Msg(debugInfo.errorSignedDistance));
    interface_.debug_yaw_vehicle.publish(conversions::doubleToFloat64Msg(debugInfo.yawVehicle));
}

void ControllerNode::setControllerParameters() {
    Parameters parameters{};
    parameters.kAngle_p = interface_.k_angular_p;
    parameters.kDistance_p = interface_.k_distance_p;
    parameters.kAngle_i = interface_.k_angular_i;
    parameters.kDistance_i = interface_.k_distance_i;
    parameters.kAngle_d = interface_.k_angular_d;
    parameters.kDistance_d = interface_.k_distance_d;
    parameters.lookAheadIndex = interface_.look_ahead_index;
    parameters.steeringAngleMax = interface_.steering_angle_max;
    parameters.wheelBase = interface_.wheel_base;
    parameters.minVelocityThreshold = interface_.min_velocity_threshold;
    parameters.control_sample_t= 1/interface_.control_loop_rate;
    parameters.k_longitude = interface_.k_longitude;
    controller_.setParameters(parameters);
}

} // namespace kal3_controller_ros_tool
