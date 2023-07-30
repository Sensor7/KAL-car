#include "kal3_controller.hpp"

int main(int argc, char* argv[]) {

    ros::init(argc, argv, "kal3_controller_node");

    kal3_controller_ros_tool::ControllerNode kal3_controller(ros::NodeHandle("~"));

    ros::spin();
    return EXIT_SUCCESS;
}
