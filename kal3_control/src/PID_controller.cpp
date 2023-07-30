#include "controller.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <tuple>

#include "internal/utils.hpp"


namespace kal3_controller {

ControlCommand Controller::getControlCommand(const Pose& vehiclePose,
                                             const Trajectory& trajectory,
                                             const double desirespeed,
                                             const bool switch2cone,
                                             const bool returnDebugInfo /* = false*/) {
    throwIfParametersNotInitialized();

    // initialize control deque
    if (!boolcontroldequeInitialized_) {
        init_control_deque();
        boolcontroldequeInitialized_ = true;
    }

    //initialize last control command
    if (!commandrecordInitialized_) {
        init_command_record();
        commandrecordInitialized_ = true;
    }

    size_t indexOfClosestPointOnTrajectory =
        utils::findIndexOfClosestPointOnTrajectory(trajectory, vehiclePose.translation());

    // Compute desired speed
    size_t indexForEstimation =
        std::clamp(indexOfClosestPointOnTrajectory, static_cast<size_t>(0), trajectory.size() - 2);
    StampedPose firstStampedPose = trajectory[indexForEstimation];
    StampedPose secondStampedPose = trajectory[indexForEstimation + 1];
    double desiredSpeed_ = desirespeed;

    // Stop the car if the speed is below the minimum speed threshold
    if (desiredSpeed_ < parameters_.minVelocityThreshold) {
        desiredSpeed_ = 0;
        reset_control_deque();
    }

    if (parameters_.lookAheadIndex < 1) {
        throw std::out_of_range("Look ahead index must be 1 or larger.");
    }

    //set the curvature to 0 when run through the cones, otherwise compute trajectory curvature using 3 points
    double curvature = 0;
    size_t indexForCurvatureEstimation  = indexForEstimation;
    if (!switch2cone) {
        indexForCurvatureEstimation = std::clamp(indexOfClosestPointOnTrajectory + parameters_.lookAheadIndex, static_cast<size_t>(1), trajectory.size() - 2);
        Position previousPoint = trajectory[indexForCurvatureEstimation-1].pose.translation();
        Position lookAheadPoint = trajectory[indexForCurvatureEstimation].pose.translation();
        Position nextPoint = trajectory[indexForCurvatureEstimation+1].pose.translation();
        curvature = utils::discreteCurvature(previousPoint, lookAheadPoint, nextPoint);
    }

    // Clamp curvature to the range of (-5,5)
    curvature = std::clamp(curvature, -5.0, 5.0);

    // Compute velocity according to curvature
    double speed = desiredSpeed_ ;
    if (trajectory.size() > 15 && abs(curvature) < 1) {
        speed = speed / (std::exp(abs(curvature)/parameters_.k_longitude));
    }

    // Apply moving average filter
    speed_deque.push_back(speed);
    speed_deque.pop_front();
    speed = std::reduce(speed_deque.begin(), speed_deque.end(), 0.0) / speed_deque.size();
    speed = std::clamp(speed, 0.0, 3.0);

    // Compute angle between road and vehicle orientation
    Eigen::Vector2d targetDirection = secondStampedPose.pose.translation() - firstStampedPose.pose.translation();
    double anglePath = std::atan2(targetDirection.y(), targetDirection.x());
    double yawVehicle = Eigen::Rotation2Dd(vehiclePose.rotation()).angle();
    double errorAngle = utils::normalizeAnglePlusMinusPi(yawVehicle - anglePath);

    // Compute signed distance between vehicle and trajectory
    double errorSignedDistance = utils::signedDistanceBetweenPointAndLine(
        vehiclePose.translation(),
        std::make_tuple(firstStampedPose.pose.translation(), secondStampedPose.pose.translation()));

    // Compute the speed of signed error distance and error angle
    double curvature_now = 0;
    if (!switch2cone) {
        curvature_now = utils::discreteCurvature(trajectory[indexForEstimation].pose.translation(), trajectory[indexForEstimation+1].pose.translation(), trajectory[indexForEstimation+2].pose.translation());
    }
    curvature_now = std::clamp(curvature_now, -5.0, 5.0);
    errorSignedDistance_speed = speed_pre*std::sin(errorAngle);
    errorAngle_speed = speed_pre*(std::tan(steeringangle_pre/parameters_.wheelBase) - curvature_now);

    // Compute steering angle using pid control law
    double u = curvature - parameters_.kDistance_p * errorSignedDistance -parameters_.kDistance_i*errorSignedDistance_sum
    -parameters_.kDistance_d*errorSignedDistance_speed- parameters_.kAngle_p * errorAngle
    -parameters_.kDistance_i*errorAngle_sum-parameters_.kDistance_d*errorAngle_speed;
    double steeringAngle = std::atan(parameters_.wheelBase * u);

    //Set flag for anti-wind up
    //saturation_flag = false;
    //if (steeringAngle<=-parameters_.steeringAngleMax || steeringAngle>=parameters_.steeringAngleMax){
      //  saturation_flag = true;
    //}

    //Clamp the steering angle
    steeringAngle = std::clamp(steeringAngle, -parameters_.steeringAngleMax, parameters_.steeringAngleMax);

    //Update integral parameters
    //if (!(saturation_flag==true && (steeringangle_pre>=0 && errorAngle<=0 || steeringangle_pre<=0 && errorAngle>=0))){
      //  errorAngle_sum += errorAngle*parameters_.control_sample_t;
    //}
    //if (!(saturation_flag==true && (steeringangle_pre>=0 && errorSignedDistance<=0 || steeringangle_pre<=0 && errorSignedDistance>=0))){
      //  errorSignedDistance_sum += errorSignedDistance*parameters_.control_sample_t;
    //}

    //update last control command
    steeringangle_pre = steeringAngle;
    speed_pre = speed;

    std::optional<ControlCommand::DebugInfo> debugInfo;
    if (returnDebugInfo) {
        debugInfo.emplace();
        debugInfo->closestPointOnTrajectory = trajectory[indexOfClosestPointOnTrajectory].pose.translation();
        debugInfo->lookAheadPoint = trajectory[indexForCurvatureEstimation].pose.translation();
        debugInfo->anglePath = anglePath;
        debugInfo->curvature = curvature;
        debugInfo->errorAngle = errorAngle;
        debugInfo->errorSignedDistance = errorSignedDistance;
        debugInfo->yawVehicle = yawVehicle;
    }

    return {speed, steeringAngle, debugInfo};
}

void Controller::throwIfParametersNotInitialized() const {
    if (!parameterInitialized_) {
        throw std::runtime_error("Controller parameters are not initialized!");
    }
}

} // namespace kal_controller
