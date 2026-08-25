#pragma once

#include <algorithm>
#include <array>
#include <cstddef>

#include "sensors/sensor_types.hpp"

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

struct ControllerKeys {
    bool w = false;
    bool a = false;
    bool s = false;
    bool d = false;
    bool q = false;
    bool e = false;
    bool r = false;
    bool f = false;
};

struct ControllerInput {
    ImuSample imu;
    float timestep_seconds = 0.0f;
    ControllerKeys keys;
};
