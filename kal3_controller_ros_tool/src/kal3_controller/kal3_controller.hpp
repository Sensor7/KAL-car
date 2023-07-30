#pragma once

#include <optional>
#include <dynamic_reconfigure/server.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <ros/time.h>
#include <tf2_ros/transform_listener.h>

#include <kal3_controller/controller.hpp>
#include <kal3_controller/types.hpp>

#include "kal3_controller_ros_tool/Kal3_ControllerInterface.h"

namespace kal3_controller_ros_tool {

using namespace kal3_controller;

class ControllerNode {
public:
    using Interface = Kal3_ControllerInterface;

    explicit ControllerNode(const ros::NodeHandle& nhPrivate);

private:
    /**
     * @brief Callback to receive and store a new trajectory.
     *
     * @param pathMsg
     */
    void trajectoryCallback(const nav_msgs::Path::ConstPtr& msg);

    /**
     * @brief Callback to receive and store updated trajectory with cones.
     *
     * @param pathMsg
     */
    void trajectoryconesCallback(const nav_msgs::Path::ConstPtr& msg_c);

    /**
     * @brief Timed callback to compute control commands from current trajectory and vehicle pose
     *
     * @param timerEvent
     */
    void controlLoopCallback(const ros::TimerEvent& timer);

    /**
     * @brief Callback to get and store the distance of stop sign
     *
     * @param sign_dist
     */
    void stopsigndisCallback(const std_msgs::Float64& sign_dist);

    /**
     * @brief Callback to get and store the distance of the nearest cone
     *
     * @param cone_dist
     */
    void conedisCallback(const std_msgs::Float64& cone_dist);

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

    /**
     * @brief Check that trajectory has been defined. Send warning and stop the car if it hasn't.
     *
     * @return True if trajectory exists, false if it doesn't
     */
    bool checkTrajectoryExists() const;

    /**
     * @brief Check that trajectory isn't older than maximum allowed age. Send warning and stop the car if it hasn't.
     *
     * @return True if trajectory time stamp is valid, false if it is too old
     */
    bool checkTrajectoryAge() const;

    /**
     * @brief Check that trajectory has the minimum required length.
     *
     * @return True if trajectory is long enough, false if it isn't
     */
    bool checkTrajectoryLength() const;

    /**
     * @brief Check the distance between stop sign and car
     *
     * @return True if distance is large enough, false if it isn't
     */
    bool checkStopsignDistance();

    /**
     * @brief Check the distance between nearest cone and car
     *
     * @return change the flag according to distance
     */
    void checkConeDistance();

    /**
     * @brief Publish a control command to stop the car.
     *
     */
    void publishStopCommand() const;

    /**
     * @brief Publish debug info.
     *
     */
    void publishDebugInfo(const ControlCommand::DebugInfo& debugInfo, const ros::Time& stamp) const;

    void setControllerParameters();

    Controller controller_{};
    std::optional<Trajectory> trajectory_{std::nullopt};
    ros::Time trajectoryStamp_{0};
    std::optional<Trajectory> trajectory_cone{std::nullopt};
    ros::Time trajectoryStamp_cone{0};

    ros::Timer controlLoopTimer_;
    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener tfListener_{tfBuffer_};

    Interface interface_;
    dynamic_reconfigure::Server<Interface::Config> reconfigureServer_;

    double sign_dist_ = 100;
    double cone_dist_ = 100;
    bool stop_flag_{false};
    int circle_num =0;
    ros::Time time_global_{0};
    bool cone_flag_{false};
    ros::Time time_cone_{0};
    double desirespeed = 1.5;
    ros::Time time_stop_{0};
};
} // namespace kal3_controller_ros_tool
