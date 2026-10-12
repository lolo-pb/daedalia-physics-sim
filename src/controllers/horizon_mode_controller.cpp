#include "controllers/horizon_mode_controller.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace {

constexpr float Pi = 3.14159265358979323846f;
constexpr float TwoPi = 2.0f * Pi;
constexpr float RotationRateRadiansPerSecond = 180.0f * Pi / 180.0f;
constexpr float YawRateRadiansPerSecond = 45.0f * Pi / 180.0f;
constexpr float ThrottleRatePerSecond = 0.25f;
constexpr float HoverThrottle = 0.4905f;
constexpr float ComplementaryGyroWeight = 0.98f;
constexpr float MinimumAccelerationSquared = 1.0e-6f;

constexpr PidConfig PitchPidConfig{
    1.0f, 0.0f, 0.1f,
    -1.0f, 1.0f,
    -0.25f, 0.25f,
};
constexpr PidConfig RollPidConfig = PitchPidConfig;
constexpr PidConfig YawPidConfig = PitchPidConfig;

float KeyAxis(bool positive, bool negative) {
    return static_cast<float>(positive) - static_cast<float>(negative);
}

float WrapAngle(float angle) {
    return std::remainder(angle, TwoPi);
}

float BlendAngle(float gyro_angle, float accelerometer_angle) {
    const float correction = WrapAngle(accelerometer_angle - gyro_angle);
    return WrapAngle(
        gyro_angle + (1.0f - ComplementaryGyroWeight) * correction);
}

void MixMotorTargets(
    ControllerOutput &output,
    float throttle,
    float pitch_correction,
    float roll_correction,
    float yaw_correction) {
    // F450 order: front-right, aft-left, front-left, aft-right.
    output.SetMotor(0, throttle + pitch_correction - roll_correction + yaw_correction);
    output.SetMotor(1, throttle - pitch_correction + roll_correction + yaw_correction);
    output.SetMotor(2, throttle + pitch_correction + roll_correction - yaw_correction);
    output.SetMotor(3, throttle - pitch_correction - roll_correction - yaw_correction);
}

} // namespace

HorizonModeController::HorizonModeController() :
    pitch_pid_(PitchPidConfig),
    roll_pid_(RollPidConfig),
    yaw_pid_(YawPidConfig) {}

void HorizonModeController::Reset() {
    pitch_pid_.Reset();
    roll_pid_.Reset();
    yaw_pid_.Reset();
    pitch_rad_ = 0.0f;
    roll_rad_ = 0.0f;
    yaw_rad_ = 0.0f;
    target_pitch_rad_ = 0.0f;
    target_roll_rad_ = 0.0f;
    target_yaw_rad_ = 0.0f;
    throttle_ = HoverThrottle;
    attitude_initialized_ = false;
}

void HorizonModeController::Update(
    const ControllerInput &input,
    ControllerOutput &output) {
    const float timestep = input.timestep_seconds;
    if (timestep <= 0.0f) {
        return;
    }

    const ControllerKeys &keys = input.keys;
    target_pitch_rad_ = WrapAngle(
        target_pitch_rad_
            + RotationRateRadiansPerSecond
                * KeyAxis(keys.s, keys.w)
                * timestep);
    target_roll_rad_ = WrapAngle(
        target_roll_rad_
            + RotationRateRadiansPerSecond
                * KeyAxis(keys.a, keys.d)
                * timestep);
    target_yaw_rad_ = WrapAngle(
        target_yaw_rad_
            + YawRateRadiansPerSecond
                * KeyAxis(keys.q, keys.e)
                * timestep);
    throttle_ = std::clamp(
        throttle_
            + ThrottleRatePerSecond
                * KeyAxis(keys.r, keys.f)
                * timestep,
        0.0f,
        1.0f);

    UpdateAttitudeEstimate(input.imu, timestep);
    const SensorVector3 &gyro = input.imu.body_gyro_rad_per_second;
    const float pitch_correction = pitch_pid_.Update(
        WrapAngle(target_pitch_rad_ - pitch_rad_), gyro.y, timestep);
    const float roll_correction = roll_pid_.Update(
        WrapAngle(target_roll_rad_ - roll_rad_), gyro.x, timestep);
    const float yaw_correction = yaw_pid_.Update(
        WrapAngle(target_yaw_rad_ - yaw_rad_), gyro.z, timestep);
    MixMotorTargets(
        output,
        throttle_,
        pitch_correction,
        roll_correction,
        yaw_correction);
}

float HorizonModeController::GetThrottle() const {
    return throttle_;
}

void HorizonModeController::UpdateAttitudeEstimate(
    const ImuSample &imu,
    float timestep) {
    const SensorVector3 &gyro = imu.body_gyro_rad_per_second;
    const SensorVector3 &acceleration =
        imu.body_specific_force_ft_per_second_squared;
    const float acceleration_squared = acceleration.x * acceleration.x
        + acceleration.y * acceleration.y
        + acceleration.z * acceleration.z;
    const bool has_accelerometer_attitude =
        acceleration_squared > MinimumAccelerationSquared;
    const float accelerometer_pitch = std::atan2(
        -acceleration.x, std::sqrt(acceleration.y * acceleration.y + acceleration.z * acceleration.z));
    const float accelerometer_roll = std::atan2(acceleration.y, -acceleration.z);

    if (!attitude_initialized_) {
        if (has_accelerometer_attitude) {
            pitch_rad_ = accelerometer_pitch;
            roll_rad_ = accelerometer_roll;
        }
        attitude_initialized_ = true;
        return;
    }

    pitch_rad_ = WrapAngle(pitch_rad_ + gyro.y * timestep);
    roll_rad_ = WrapAngle(roll_rad_ + gyro.x * timestep);
    yaw_rad_ = WrapAngle(yaw_rad_ + gyro.z * timestep);
    if (has_accelerometer_attitude) {
        pitch_rad_ = BlendAngle(pitch_rad_, accelerometer_pitch);
        roll_rad_ = BlendAngle(roll_rad_, accelerometer_roll);
    }
}
