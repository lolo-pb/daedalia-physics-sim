#include "controllers/manual_controller.hpp"

#include <algorithm>

namespace {
float Axis(bool positive, bool negative) {
    return static_cast<float>(positive) - static_cast<float>(negative);
}
}

void ManualController::Reset() { throttle_ = 0.0f; flaps_ = 0.0f; }

void ManualController::Update(const ControllerInput &input, ControllerOutput &output) {
    if (input.timestep_seconds <= 0.0f) return;
    const auto &keys = input.keys;
    throttle_ = std::clamp(throttle_ + 0.25f * input.timestep_seconds * Axis(keys.r, keys.f), 0.0f, 1.0f);
    flaps_ = std::clamp(flaps_ + 0.5f * input.timestep_seconds * Axis(keys.g, keys.h), 0.0f, 1.0f);
    output.throttle = throttle_;
    output.roll = Axis(keys.d, keys.a);
    output.pitch = Axis(keys.s, keys.w);
    output.yaw = Axis(keys.e, keys.q);
    output.flaps = flaps_;
    output.left_brake = output.right_brake = keys.b ? 1.0f : 0.0f;
}
