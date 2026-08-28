#pragma once

#include "controllers/controller_io.hpp"

struct AttitudeSetpoint {
    // Body axes are pitch about +X, yaw about +Y, and roll about +Z.
    // Yaw is relative to the heading at the most recent Reset().
    float pitch_rad = 0.0f;
    float roll_rad = 0.0f;
    float yaw_rad = 0.0f;
    float throttle = 0.0f;
};

class AttitudeController {
public:
    void Reset();
    void Update(
        const ControllerInput &input,
        const AttitudeSetpoint &setpoint,
        MotorCommands &motor_commands);

private:
    void UpdateAttitudeEstimate(const ImuSample &imu, float timestep);

    float pitch_rad_ = 0.0f;
    float roll_rad_ = 0.0f;
    float yaw_rad_ = 0.0f;
    float pitch_integral_ = 0.0f;
    float roll_integral_ = 0.0f;
    float yaw_integral_ = 0.0f;
    bool attitude_initialized_ = false;
};
