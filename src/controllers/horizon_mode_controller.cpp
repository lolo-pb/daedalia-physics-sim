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
constexpr float HoverThrottle = 0.70f;
constexpr float ComplementaryGyroWeight = 0.98f;
constexpr float MinimumAccelerationSquared = 1.0e-6f;
constexpr std::size_t FrontLeftMotor = 0;
constexpr std::size_t FrontRightMotor = 1;
constexpr std::size_t RearRightMotor = 2;
constexpr std::size_t RearLeftMotor = 3;

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
    MotorCommands &motor_commands,
    float throttle,
    float pitch_correction,
    float roll_correction,
    float yaw_correction) {
    motor_commands.SetMotor(
        FrontLeftMotor,
        throttle + pitch_correction - roll_correction + yaw_correction);
    motor_commands.SetMotor(
        FrontRightMotor,
        throttle + pitch_correction + roll_correction - yaw_correction);
    motor_commands.SetMotor(
        RearRightMotor,
        throttle - pitch_correction + roll_correction + yaw_correction);
    motor_commands.SetMotor(
        RearLeftMotor,
        throttle - pitch_correction - roll_correction - yaw_correction);
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
    MotorCommands &motor_commands) {
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
        WrapAngle(target_pitch_rad_ - pitch_rad_), gyro.x, timestep);
    const float roll_correction = roll_pid_.Update(
        WrapAngle(target_roll_rad_ - roll_rad_), gyro.z, timestep);
    const float yaw_correction = yaw_pid_.Update(
        WrapAngle(target_yaw_rad_ - yaw_rad_), gyro.y, timestep);
    MixMotorTargets(
        motor_commands,
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
        imu.body_specific_force_meters_per_second_squared;
    const float acceleration_squared = acceleration.x * acceleration.x
        + acceleration.y * acceleration.y
        + acceleration.z * acceleration.z;
    const bool has_accelerometer_attitude =
        acceleration_squared > MinimumAccelerationSquared;
    const float accelerometer_pitch =
        std::atan2(-acceleration.z, acceleration.y);
    const float accelerometer_roll =
        std::atan2(acceleration.x, acceleration.y);

    if (!attitude_initialized_) {
        if (has_accelerometer_attitude) {
            pitch_rad_ = accelerometer_pitch;
            roll_rad_ = accelerometer_roll;
        }
        attitude_initialized_ = true;
        return;
    }

    pitch_rad_ = WrapAngle(pitch_rad_ + gyro.x * timestep);
    roll_rad_ = WrapAngle(roll_rad_ + gyro.z * timestep);
    yaw_rad_ = WrapAngle(yaw_rad_ + gyro.y * timestep);
    if (has_accelerometer_attitude) {
        pitch_rad_ = BlendAngle(pitch_rad_, accelerometer_pitch);
        roll_rad_ = BlendAngle(roll_rad_, accelerometer_roll);
    }
}
