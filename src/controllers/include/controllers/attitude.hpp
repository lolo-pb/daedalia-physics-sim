#pragma once

#include "controllers/controller_io.hpp"
#include "controllers/pid.hpp"

enum class AttitudeYawReference {
    RelativeToReset,
    MagneticHeading,
};

struct AttitudeConfig {
    PidConfig pitch{
        1.0f, 0.0f, 0.1f,
        -1.0f, 1.0f,
        -0.25f, 0.25f,
    };
    PidConfig roll{
        1.0f, 0.0f, 0.1f,
        -1.0f, 1.0f,
        -0.25f, 0.25f,
    };
    PidConfig yaw{
        1.0f, 0.0f, 0.1f,
        -1.0f, 1.0f,
        -0.25f, 0.25f,
    };
    AttitudeYawReference yaw_reference =
        AttitudeYawReference::RelativeToReset;
};

struct AttitudeSetpoint {
    // Body axes are pitch about +X, yaw about +Y, and roll about +Z.
    // The configured yaw reference is relative to Reset or magnetic heading.
    float pitch_rad = 0.0f;
    float roll_rad = 0.0f;
    float yaw_rad = 0.0f;
};

struct AttitudeCorrection {
    float pitch = 0.0f;
    float roll = 0.0f;
    float yaw = 0.0f;
};

class Attitude {
public:
    explicit Attitude(const AttitudeConfig &config = {});

    void Reset();
    AttitudeCorrection Update(
        const ControllerInput &input,
        const AttitudeSetpoint &setpoint);

private:
    void UpdateEstimate(
        const ControllerInput &input,
        float timestep);

    AttitudeYawReference yaw_reference_;
    Pid pitch_pid_;
    Pid roll_pid_;
    Pid yaw_pid_;
    float pitch_rad_ = 0.0f;
    float roll_rad_ = 0.0f;
    float yaw_rad_ = 0.0f;
    bool initialized_ = false;
};
