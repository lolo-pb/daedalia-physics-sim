#pragma once

#include "controllers/controller_io.hpp"
#include "controllers/pid.hpp"

class PositionHoldController {
public:
    PositionHoldController();

    void Reset();
    void Update(const ControllerInput &input, MotorCommands &motor_commands);

private:
    void CaptureTarget(const ControllerInput &input);
    void UpdateAttitudeEstimate(const ControllerInput &input);

    Pid pitch_pid_;
    Pid roll_pid_;
    Pid yaw_pid_;
    Pid altitude_pid_;
    float target_world_x_meters_ = 0.0f;
    float target_world_z_meters_ = 0.0f;
    float target_altitude_meters_ = 0.0f;
    float target_heading_rad_ = 0.0f;
    float pitch_rad_ = 0.0f;
    float roll_rad_ = 0.0f;
    float yaw_rad_ = 0.0f;
    bool target_initialized_ = false;
};
