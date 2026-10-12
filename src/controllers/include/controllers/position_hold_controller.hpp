#pragma once

#include "controllers/controller_io.hpp"
#include "controllers/pid.hpp"

class PositionHoldController {
public:
    PositionHoldController();

    void Reset();
    void Update(const ControllerInput &input, ControllerOutput &output);

private:
    void CaptureTarget(const ControllerInput &input);
    void InitializeLevelFromGravity(const ControllerInput &input);
    void UpdateAttitudeEstimate(const ControllerInput &input);
    bool RecoverLevel(const ControllerInput &input, ControllerOutput &output);

    Pid north_position_pid_;
    Pid east_position_pid_;
    Pid north_velocity_pid_;
    Pid east_velocity_pid_;
    Pid altitude_pid_;
    Pid pitch_pid_;
    Pid roll_pid_;
    Pid yaw_pid_;
    Pid emergency_pitch_pid_;
    Pid emergency_roll_pid_;
    float target_north_ft_ = 0.0f;
    float target_east_ft_ = 0.0f;
    float target_altitude_ft_ = 0.0f;
    float target_heading_rad_ = 0.0f;
    float pitch_rad_ = 0.0f;
    float roll_rad_ = 0.0f;
    float yaw_rad_ = 0.0f;
    bool attitude_initialized_ = false;
    bool position_hold_initialized_ = false;
};
