#pragma once

#include "controllers/attitude_controller.hpp"

class HorizonModeController {
public:
    void Reset();
    void Update(const ControllerInput &input, MotorCommands &motor_commands);

    float GetThrottle() const;

private:
    AttitudeController attitude_controller_;
    AttitudeSetpoint setpoint_{.throttle = 0.70f};
};
