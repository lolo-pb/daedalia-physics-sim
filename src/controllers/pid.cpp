#include "controllers/pid.hpp"

#include <algorithm>

Pid::Pid(const PidConfig &config) : config_(config) {}

void Pid::Reset() {
    integral_ = 0.0f;
}

float Pid::Update(float error, float measured_rate, float timestep) {
    if (timestep <= 0.0f) {
        return 0.0f;
    }

    integral_ = std::clamp(
        integral_ + error * timestep,
        config_.minimum_integral,
        config_.maximum_integral);
    return std::clamp(
        config_.proportional * error
            + config_.integral * integral_
            - config_.derivative * measured_rate,
        config_.minimum_output,
        config_.maximum_output);
}
