#pragma once

#include "controllers/controller_io.hpp"
#include "controllers/pid.hpp"

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
    float pitch_rad_ = 0.0f;
    float roll_rad_ = 0.0f;
    float yaw_rad_ = 0.0f;
    float target_yaw_rad_ = 0.0f;
    float throttle_ = 0.70f;
    bool attitude_initialized_ = false;
};
