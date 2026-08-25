#pragma once

#include "controllers/controller_io.hpp"

class PositionHoldController {
public:
    void Reset();
    void Update(const ControllerInput &input, TargetDrone &drone);

private:
    void CaptureTarget(const ControllerInput &input);
    void UpdateAttitudeEstimate(const ControllerInput &input);

    float target_world_x_meters_ = 0.0f;
    float target_world_z_meters_ = 0.0f;
    float target_altitude_meters_ = 0.0f;
    float target_heading_rad_ = 0.0f;
    float pitch_rad_ = 0.0f;
    float roll_rad_ = 0.0f;
    float yaw_rad_ = 0.0f;
    float pitch_integral_ = 0.0f;
    float roll_integral_ = 0.0f;
    float yaw_integral_ = 0.0f;
    float altitude_integral_ = 0.0f;
    bool target_initialized_ = false;
};
