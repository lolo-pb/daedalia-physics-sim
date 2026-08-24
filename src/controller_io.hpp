#pragma once

#include <algorithm>
#include <array>
#include <cstddef>

enum class MotorId {
    FrontLeft,
    FrontRight,
    RearRight,
    RearLeft,
};

class TargetDrone {
public:
    void SetMotorTarget(MotorId id, float target) {
        targets_[static_cast<std::size_t>(id)] = std::clamp(target, 0.0f, 1.0f);
    }

    float GetMotorTarget(MotorId id) const {
        return targets_[static_cast<std::size_t>(id)];
    }

private:
    std::array<float, 4> targets_{};
};

struct ImuVector3 {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
};

struct ImuSample {
    double timestamp_seconds = 0.0;
    ImuVector3 body_gyro_rad_per_second;
    ImuVector3 body_specific_force_meters_per_second_squared;
};

struct ControllerInput {
    ImuSample imu;
    float timestep_seconds = 0.0f;
};
