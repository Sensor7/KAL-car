# KAL TRAJECTORY PLANNER ROS TOOL

ROS wrapper for the kal_trajectory_planner package

## What is this package doing?

This ROS wrapper handles all ROS related functionality to call the library implementation of the planner in the kal_trajectory_planner package.
The provided node will listen to path messages and find the current vehicle pose in the tf tree, convert them to Eigen data types and pass both to the library.
The returned trajectory will then be converted to a ROS message and published.
Additionally, the wrapper handles the parameters.

## Usage

Include the `trajectory_planner_node.launch` file in a meta package launching the autonomous stack and set the required parameters.

For testing and development purposes, there is also a standalone launch file available.

## Test

Check out the rostest in the test directory of this package.
It starts the node, publishes a mocked path and vehicle pose and makes sure the computed trajectory meets the expectations.

Since testing with ROS can be quite cumbersome, most of the functionality is tested in kal_trajectory_planner directory.