#pragma once

#include "controllers/controller_io.hpp"
#include "controllers/pid.hpp"
#include "controllers/quaternion.hpp"

class TricopterAngleModeController {
public:
    TricopterAngleModeController();

    void Reset();
    void Update(const ControllerInput &input, MotorCommands &motor_commands);

    float GetThrottle() const;

private:
    void UpdateAttitudeEstimate(const ImuSample &imu, float timestep);

    Pid pitch_pid_;
    Pid roll_pid_;
    Quaternion orientation_;
    float throttle_ = 0.572f;
    bool attitude_initialized_ = false;
};
