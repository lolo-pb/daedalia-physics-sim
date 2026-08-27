#include "controllers/horizon_mode_controller.hpp"

#include <algorithm>
#include <cmath>

namespace {

constexpr float Pi = 3.14159265358979323846f;
constexpr float RotationRateRadiansPerSecond = 180.0f * Pi / 180.0f;
constexpr float YawRateRadiansPerSecond = 45.0f * Pi / 180.0f;
constexpr float ThrottleRatePerSecond = 0.25f;
constexpr float HoverThrottle = 0.70f;

float KeyAxis(bool positive, bool negative) {
    return static_cast<float>(positive) - static_cast<float>(negative);
}

} // namespace

void HorizonModeController::Reset() {
    attitude_controller_.Reset();
    setpoint_ = AttitudeSetpoint{.throttle = HoverThrottle};
}

void HorizonModeController::Update(
    const ControllerInput &input,
    TargetDrone &drone) {
    const ControllerKeys &keys = input.keys;

    if (input.timestep_seconds > 0.0f) {
        setpoint_.pitch_rad = std::remainder(
            setpoint_.pitch_rad
                + RotationRateRadiansPerSecond
                    * KeyAxis(keys.s, keys.w)
                    * input.timestep_seconds,
            2.0f * Pi);
        setpoint_.roll_rad = std::remainder(
            setpoint_.roll_rad
                + RotationRateRadiansPerSecond
                    * KeyAxis(keys.a, keys.d)
                    * input.timestep_seconds,
            2.0f * Pi);
        setpoint_.yaw_rad = std::remainder(
            setpoint_.yaw_rad
                + YawRateRadiansPerSecond
                    * KeyAxis(keys.q, keys.e)
                    * input.timestep_seconds,
            2.0f * Pi);
        setpoint_.throttle = std::clamp(
            setpoint_.throttle
                + ThrottleRatePerSecond
                    * KeyAxis(keys.r, keys.f)
                    * input.timestep_seconds,
            0.0f,
            1.0f);
    }

    attitude_controller_.Update(input, setpoint_, drone);
}

float HorizonModeController::GetThrottle() const {
    return setpoint_.throttle;
}
