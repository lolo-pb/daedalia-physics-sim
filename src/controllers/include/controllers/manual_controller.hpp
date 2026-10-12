#pragma once

#include "controllers/controller_io.hpp"

class ManualController {
public:
    void Reset();
    void Update(const ControllerInput &input, ControllerOutput &output);
    float GetThrottle() const { return throttle_; }
private:
    float throttle_ = 0.0f;
    float flaps_ = 0.0f;
};
