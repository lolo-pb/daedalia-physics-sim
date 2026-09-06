#include "controllers/attitude.hpp"

#include <cmath>

namespace {

constexpr float Pi = 3.14159265358979323846f;
constexpr float TwoPi = 2.0f * Pi;
constexpr float ComplementaryGyroWeight = 0.98f;
constexpr float MinimumAccelerationSquared = 1.0e-6f;

float WrapAngle(float angle) {
    return std::remainder(angle, TwoPi);
}

float BlendAngle(float gyro_angle, float measured_angle) {
    const float correction = WrapAngle(measured_angle - gyro_angle);
    return WrapAngle(
        gyro_angle + (1.0f - ComplementaryGyroWeight) * correction);
}

float MagneticHeading(const MagnetometerSample &magnetometer) {
    const SensorVector3 &field = magnetometer.body_magnetic_field_microteslas;
    return std::atan2(field.x, -field.z);
}

} // namespace

Attitude::Attitude(const AttitudeConfig &config) :
    yaw_reference_(config.yaw_reference),
    pitch_pid_(config.pitch),
    roll_pid_(config.roll),
    yaw_pid_(config.yaw) {}

void Attitude::Reset() {
    pitch_rad_ = 0.0f;
    roll_rad_ = 0.0f;
    yaw_rad_ = 0.0f;
    pitch_pid_.Reset();
    roll_pid_.Reset();
    yaw_pid_.Reset();
    initialized_ = false;
}

AttitudeCorrection Attitude::Update(
    const ControllerInput &input,
    const AttitudeSetpoint &setpoint) {
    const float timestep = input.timestep_seconds;
    if (timestep <= 0.0f) {
        return {};
    }

    const SensorVector3 &gyro = input.imu.body_gyro_rad_per_second;
    UpdateEstimate(input, timestep);

    return {
        pitch_pid_.Update(
            WrapAngle(setpoint.pitch_rad - pitch_rad_),
            gyro.x,
            timestep),
        roll_pid_.Update(
            WrapAngle(setpoint.roll_rad - roll_rad_),
            gyro.z,
            timestep),
        yaw_pid_.Update(
            WrapAngle(setpoint.yaw_rad - yaw_rad_),
            gyro.y,
            timestep),
    };
}

void Attitude::UpdateEstimate(
    const ControllerInput &input,
    float timestep) {
    const SensorVector3 &gyro = input.imu.body_gyro_rad_per_second;
    const SensorVector3 &acceleration =
        input.imu.body_specific_force_meters_per_second_squared;
    const float acceleration_squared = acceleration.x * acceleration.x
        + acceleration.y * acceleration.y
        + acceleration.z * acceleration.z;
    const bool has_accelerometer_attitude =
        acceleration_squared > MinimumAccelerationSquared;
    const float accelerometer_pitch =
        std::atan2(-acceleration.z, acceleration.y);
    const float accelerometer_roll =
        std::atan2(acceleration.x, acceleration.y);

    if (!initialized_) {
        if (has_accelerometer_attitude) {
            pitch_rad_ = accelerometer_pitch;
            roll_rad_ = accelerometer_roll;
        }
        if (yaw_reference_ == AttitudeYawReference::MagneticHeading) {
            yaw_rad_ = MagneticHeading(input.magnetometer);
        }
        initialized_ = true;
        return;
    }

    pitch_rad_ = WrapAngle(pitch_rad_ + gyro.x * timestep);
    roll_rad_ = WrapAngle(roll_rad_ + gyro.z * timestep);
    yaw_rad_ = WrapAngle(yaw_rad_ + gyro.y * timestep);

    if (has_accelerometer_attitude) {
        pitch_rad_ = BlendAngle(pitch_rad_, accelerometer_pitch);
        roll_rad_ = BlendAngle(roll_rad_, accelerometer_roll);
    }
    if (yaw_reference_ == AttitudeYawReference::MagneticHeading) {
        yaw_rad_ = BlendAngle(
            yaw_rad_,
            MagneticHeading(input.magnetometer));
    }
}
