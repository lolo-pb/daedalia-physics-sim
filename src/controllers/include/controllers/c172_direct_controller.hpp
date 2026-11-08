#pragma once

#include "controllers/controller_io.hpp"

// Temporary keyboard controller. It writes C172 actuator commands directly;
// it contains no aircraft stabilization or flight-control logic.
class C172DirectController {
public:
    void Reset();
    void Update(const ControllerInput &input, ControllerOutput &output);
    float GetThrottle() const { return throttle_; }

private:
    float throttle_ = 0.0f;
    float flaps_ = 0.0f;
};
