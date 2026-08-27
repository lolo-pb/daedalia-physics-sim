#include "controllers/angle_mode_controller.hpp"

#include <algorithm>
#include <cmath>

namespace {

constexpr float Pi = 3.14159265358979323846f;
constexpr float TiltRadians = 10.0f * Pi / 180.0f;
constexpr float YawRateRadiansPerSecond = 45.0f * Pi / 180.0f;
constexpr float ThrottleRatePerSecond = 0.25f;
constexpr float HoverThrottle = 0.70f;

float KeyAxis(bool positive, bool negative) {
    return static_cast<float>(positive) - static_cast<float>(negative);
}

} // namespace

void AngleModeController::Reset() {
    attitude_controller_.Reset();
    setpoint_ = AttitudeSetpoint{.throttle = HoverThrottle};
}

void AngleModeController::Update(
    const ControllerInput &input,
    TargetDrone &drone) {
    const ControllerKeys &keys = input.keys;

    setpoint_.pitch_rad = TiltRadians * KeyAxis(keys.s, keys.w);
    setpoint_.roll_rad = TiltRadians * KeyAxis(keys.a, keys.d);

    if (input.timestep_seconds > 0.0f) {
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

float AngleModeController::GetThrottle() const {
    return setpoint_.throttle;
}
