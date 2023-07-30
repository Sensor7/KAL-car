#pragma once

#include "types.hpp"

namespace kal3_controller {

class Controller {
public:
    ControlCommand getControlCommand(const Pose& vehiclePose,
                                     const Trajectory& trajectory,
                                     const double desirespeed,
                                     bool switch2cone,
                                     bool returnDebugInfo = false);

    void setParameters(const Parameters& parameters) {
        parameters_ = parameters;
        parameterInitialized_ = true;
    }

    void throwIfParametersNotInitialized() const;

    double errorSignedDistance_speed=0, errorSignedDistance_sum=0, errorAngle_speed=0, errorAngle_sum=0;
    double speed_pre, steeringangle_pre;

    std::deque<double> speed_deque;
    std::deque<double> steering_deque;

    bool saturation_flag{false};

    void init_pid_params() {
        errorSignedDistance_sum = 0;
        errorSignedDistance_speed=0;
        errorAngle_sum = 0;
        errorAngle_speed=0;
        saturation_flag = false;

        pidparamsInitialized_ =true;
    }

    void init_command_record() {
        speed_pre = 0;
        steeringangle_pre =0;
    }

    void init_control_deque() {
        speed_deque.push_back(0);
        speed_deque.push_back(0);
        speed_deque.push_back(0);
        speed_deque.push_back(0);
        speed_deque.push_back(0);
    }

    void reset_control_deque() {
        speed_deque.pop_front();
        speed_deque.pop_front();
        speed_deque.pop_front();
        speed_deque.pop_front();
        speed_deque.pop_front();

        speed_deque.push_back(0);
        speed_deque.push_back(0);
        speed_deque.push_back(0);
        speed_deque.push_back(0);
        speed_deque.push_back(0);
    }

private:
    Parameters parameters_;
    bool parameterInitialized_{false};
    bool boolcontroldequeInitialized_{false};
    bool pidparamsInitialized_{false};
    bool commandrecordInitialized_{false};
};

} // namespace kal_controller
