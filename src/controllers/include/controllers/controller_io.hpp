#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

#include "sensors/sensor_types.hpp"

class MotorCommands {
public:
    explicit MotorCommands(std::size_t motor_count) : targets_(motor_count) {}

    void Clear() {
        std::fill(targets_.begin(), targets_.end(), 0.0f);
    }

    void SetMotor(std::size_t index, float target) {
        if (index >= targets_.size()) {
            return;
        }
        targets_[index] = std::isfinite(target)
            ? std::clamp(target, 0.0f, 1.0f)
            : 0.0f;
    }

    float GetMotor(std::size_t index) const {
        return index < targets_.size() ? targets_[index] : 0.0f;
    }

private:
    std::vector<float> targets_;
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
    bool x = false;
};

struct ControllerInput {
    ImuSample imu;
    GpsSample gps;
    BarometerSample barometer;
    MagnetometerSample magnetometer;
    float timestep_seconds = 0.0f;
    ControllerKeys keys;
};
