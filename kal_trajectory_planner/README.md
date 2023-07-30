# KAL TRAJECTORY PLANNER

Simple library implementation of a trajectory planner for the Kognitive Automobile Lab.

## How it works

The trajectory planner will use a given path and the vehicle pose to return a trajectory.
The trajectory always contains a fixed number of points, starts from the closest point on the path is of configurable length.
Using the points from the path, a polynomial of configurable degree will be estimated, the trajectory points are taken from that polynomial.
The desired velocity is implied by the time stamps and can be configured.
Near the end of the path, the speed will be reduced, the trajectory will get shorter but will still contain the defined number of points.

## Usage

Instantiate the TrajectoryPlanner class and set the path at least once.
To actually compute the trajectory, call `computeTrajectory`.

Take a look at the unit tests or the ROS wrapper for exemplary usage.