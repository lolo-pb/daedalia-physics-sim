#pragma once

#include "controllers/controller_io.hpp"
#include "controllers/pid.hpp"
#include "controllers/quaternion.hpp"

class AngleModeController {
public:
    AngleModeController();

    void Reset();
    void Update(const ControllerInput &input, MotorCommands &motor_commands);

    float GetThrottle() const;

private:
    void UpdateAttitudeEstimate(const ImuSample &imu, float timestep);

    Pid pitch_pid_;
    Pid roll_pid_;
    Pid yaw_pid_;
    Quaternion orientation_;
    float target_yaw_rad_ = 0.0f;
    float throttle_ = 0.4905f;
    bool attitude_initialized_ = false;
};
