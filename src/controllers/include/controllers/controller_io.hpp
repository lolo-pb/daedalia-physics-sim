#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

#include "sensors/sensor_types.hpp"

// Single command contract for all controllers. Motor mapping is reserved for
// the later, approved legacy-controller integration.
class ControllerOutput {
public:
    explicit ControllerOutput(std::size_t motor_count = 0) : motors_(motor_count) {}
    void Clear() {
        std::fill(motors_.begin(), motors_.end(), 0.0f);
        throttle = roll = pitch = yaw = flaps = left_brake = right_brake = 0.0f;
    }
    void SetMotor(std::size_t index, float target) {
        if (index < motors_.size()) motors_[index] = Bound(target, 0.0f, 1.0f);
    }
    float GetMotor(std::size_t index) const {
        return index < motors_.size() ? motors_[index] : 0.0f;
    }
    std::size_t GetMotorCount() const { return motors_.size(); }
    void Normalize() {
        throttle = Bound(throttle, 0, 1);
        roll = Bound(roll, -1, 1);
        pitch = Bound(pitch, -1, 1);
        yaw = Bound(yaw, -1, 1);
        flaps = Bound(flaps, 0, 1);
        left_brake = Bound(left_brake, 0, 1);
        right_brake = Bound(right_brake, 0, 1);
    }
    float throttle = 0.0f;
    float roll = 0.0f;
    float pitch = 0.0f;
    float yaw = 0.0f;
    float flaps = 0.0f;
    float left_brake = 0.0f;
    float right_brake = 0.0f;
private:
    static float Bound(float value, float low, float high) {
        return std::isfinite(value) ? std::clamp(value, low, high) : 0.0f;
    }
    std::vector<float> motors_;
};

// Legacy sources remain unbuilt until their individual migration is approved.
using MotorCommands = ControllerOutput;

struct ControllerKeys {
    bool w = false, a = false, s = false, d = false;
    bool q = false, e = false, r = false, f = false, x = false;
    bool g = false, h = false, b = false;
};
struct ControllerInput {
    ImuSample imu;
    GpsSample gps;
    BarometerSample barometer;
    MagnetometerSample magnetometer;
    float timestep_seconds = 0.0f;
    ControllerKeys keys;
};
