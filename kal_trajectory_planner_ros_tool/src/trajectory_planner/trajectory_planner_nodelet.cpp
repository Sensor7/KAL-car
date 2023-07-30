#include <memory>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include "trajectory_planner.hpp"

namespace kal_trajectory_planner_ros_tool {

class TrajectoryPlannerNodelet : public nodelet::Nodelet {

    inline void onInit() override {
        impl_ = std::make_unique<TrajectoryPlannerNode>(getPrivateNodeHandle());
    }
    std::unique_ptr<TrajectoryPlannerNode> impl_;
};
} // namespace kal_trajectory_planner_ros_tool

PLUGINLIB_EXPORT_CLASS(kal_trajectory_planner_ros_tool::TrajectoryPlannerNodelet, nodelet::Nodelet);
