#include "controllers/position_hold_controller.hpp"

#include <algorithm>
#include <cmath>

/*
Position Hold behavior:
- The first update captures GPS north/east, barometric altitude, and magnetic heading.
- W/S and A/D move the horizontal target; R/F change the altitude target; Q/E turn the heading target.
- GPS position and velocity request a limited horizontal acceleration and tilt.
- Barometric altitude and GPS vertical velocity command throttle around the F450 hover throttle.
- IMU and magnetometer attitude estimates feed PIDs that write the four F450 motor targets.
*/
namespace {

constexpr float Pi = 3.14159265358979323846f;
constexpr float TwoPi = 2.0f * Pi;
constexpr float GravityFtPerSecondSquared = 32.174f;
constexpr float FeetPerMeter = 3.28084f;
constexpr float SeaLevelPressurePsf = 2116.21662f;
constexpr float AtmosphericScaleHeightFt = 8434.5f * FeetPerMeter;
constexpr float ComplementaryGyroWeight = 0.995f;
constexpr float MinimumAccelerationSquared = 1.0e-6f;
constexpr float HoverThrottle = 0.35f;
constexpr float HorizontalPositionGain = 0.2f;
constexpr float HorizontalVelocityGain = 0.75f;
constexpr float MaximumHorizontalApproachSpeedFtPerSecond = 1.0f * FeetPerMeter;
constexpr float MaximumHorizontalAccelerationFtPerSecondSquared = 0.75f * FeetPerMeter;
constexpr float MaximumTiltRadians = 7.0f * Pi / 180.0f;
constexpr float AltitudeProportionalGain = 0.03f / FeetPerMeter;
constexpr float AltitudeIntegralGain = 0.005f / FeetPerMeter;
constexpr float AltitudeVelocityGain = 0.10f / FeetPerMeter;
constexpr float AltitudeIntegralLimit = 2.0f * FeetPerMeter;
constexpr float MinimumThrottle = 0.05f;
constexpr float MaximumThrottle = 0.65f;
constexpr float HorizontalTargetSpeedFtPerSecond = FeetPerMeter;
constexpr float AltitudeTargetSpeedFtPerSecond = 0.5f * FeetPerMeter;
constexpr float YawTargetRateRadiansPerSecond = 45.0f * Pi / 180.0f;

constexpr PidConfig PitchPidConfig{0.35f, 0.0f, 0.10f, -1.0f, 1.0f, -0.10f, 0.10f};
constexpr PidConfig RollPidConfig = PitchPidConfig;
constexpr PidConfig YawPidConfig = PitchPidConfig;
constexpr PidConfig AltitudePidConfig{
    AltitudeProportionalGain, AltitudeIntegralGain, AltitudeVelocityGain,
    -AltitudeIntegralLimit, AltitudeIntegralLimit,
    MinimumThrottle - HoverThrottle, MaximumThrottle - HoverThrottle,
};

float KeyAxis(bool positive, bool negative) {
    return static_cast<float>(positive) - static_cast<float>(negative);
}

float WrapAngle(float angle) { return std::remainder(angle, TwoPi); }

float BlendAngle(float gyro_angle, float measured_angle) {
    return WrapAngle(gyro_angle + (1.0f - ComplementaryGyroWeight)
        * WrapAngle(measured_angle - gyro_angle));
}

float MagneticHeading(const MagnetometerSample &magnetometer) {
    const SensorVector3 &field = magnetometer.body_magnetic_field_microteslas;
    return std::atan2(field.y, field.x);
}

float BarometricAltitudeFt(const BarometerSample &barometer) {
    return -AtmosphericScaleHeightFt * std::log(barometer.pressure_psf / SeaLevelPressurePsf);
}

void MixMotorTargets(ControllerOutput &output, float throttle,
                     float pitch_correction, float roll_correction,
                     float yaw_correction) {
    // F450 order: front-right, aft-left, front-left, aft-right.
    output.SetMotor(0, throttle + pitch_correction - roll_correction + yaw_correction);
    output.SetMotor(1, throttle - pitch_correction + roll_correction + yaw_correction);
    output.SetMotor(2, throttle + pitch_correction + roll_correction - yaw_correction);
    output.SetMotor(3, throttle - pitch_correction - roll_correction - yaw_correction);
}

} // namespace

PositionHoldController::PositionHoldController()
    : pitch_pid_(PitchPidConfig), roll_pid_(RollPidConfig), yaw_pid_(YawPidConfig),
      altitude_pid_(AltitudePidConfig) {}

void PositionHoldController::Reset() {
    target_north_ft_ = 0.0f;
    target_east_ft_ = 0.0f;
    target_altitude_ft_ = 0.0f;
    target_heading_rad_ = 0.0f;
    pitch_rad_ = 0.0f;
    roll_rad_ = 0.0f;
    yaw_rad_ = 0.0f;
    pitch_pid_.Reset();
    roll_pid_.Reset();
    yaw_pid_.Reset();
    altitude_pid_.Reset();
    target_initialized_ = false;
}

void PositionHoldController::Update(const ControllerInput &input, ControllerOutput &output) {
    const float timestep = input.timestep_seconds;
    if (timestep <= 0.0f) return;
    if (!target_initialized_) CaptureTarget(input);
    else UpdateAttitudeEstimate(input);

    const ControllerKeys &keys = input.keys;
    const float forward_input = KeyAxis(keys.w, keys.s);
    const float right_input = KeyAxis(keys.d, keys.a);
    const float heading_sine = std::sin(target_heading_rad_);
    const float heading_cosine = std::cos(target_heading_rad_);
    target_north_ft_ += HorizontalTargetSpeedFtPerSecond
        * (forward_input * heading_cosine - right_input * heading_sine) * timestep;
    target_east_ft_ += HorizontalTargetSpeedFtPerSecond
        * (forward_input * heading_sine + right_input * heading_cosine) * timestep;
    target_altitude_ft_ += AltitudeTargetSpeedFtPerSecond * KeyAxis(keys.r, keys.f) * timestep;
    target_heading_rad_ = WrapAngle(target_heading_rad_
        + YawTargetRateRadiansPerSecond * KeyAxis(keys.q, keys.e) * timestep);

    const GpsSample &gps = input.gps;
    float desired_north_velocity = HorizontalPositionGain / HorizontalVelocityGain
        * (target_north_ft_ - gps.position_ned_ft.x);
    float desired_east_velocity = HorizontalPositionGain / HorizontalVelocityGain
        * (target_east_ft_ - gps.position_ned_ft.y);
    const float desired_horizontal_speed = std::hypot(desired_north_velocity, desired_east_velocity);
    if (desired_horizontal_speed > MaximumHorizontalApproachSpeedFtPerSecond) {
        const float scale = MaximumHorizontalApproachSpeedFtPerSecond / desired_horizontal_speed;
        desired_north_velocity *= scale;
        desired_east_velocity *= scale;
    }

    float north_acceleration = HorizontalVelocityGain
        * (desired_north_velocity - gps.velocity_ned_ft_per_second.x);
    float east_acceleration = HorizontalVelocityGain
        * (desired_east_velocity - gps.velocity_ned_ft_per_second.y);
    const float horizontal_acceleration = std::hypot(north_acceleration, east_acceleration);
    if (horizontal_acceleration > MaximumHorizontalAccelerationFtPerSecondSquared) {
        const float scale = MaximumHorizontalAccelerationFtPerSecondSquared / horizontal_acceleration;
        north_acceleration *= scale;
        east_acceleration *= scale;
    }

    const float forward_acceleration = north_acceleration * heading_cosine + east_acceleration * heading_sine;
    const float right_acceleration = -north_acceleration * heading_sine + east_acceleration * heading_cosine;
    const float desired_pitch = std::clamp(-std::atan2(forward_acceleration, GravityFtPerSecondSquared),
        -MaximumTiltRadians, MaximumTiltRadians);
    const float desired_roll = std::clamp(-std::atan2(right_acceleration, GravityFtPerSecondSquared),
        -MaximumTiltRadians, MaximumTiltRadians);

    const float altitude_error = target_altitude_ft_ - BarometricAltitudeFt(input.barometer);
    const float throttle = HoverThrottle + altitude_pid_.Update(
        altitude_error, -gps.velocity_ned_ft_per_second.z, timestep);
    const SensorVector3 &gyro = input.imu.body_gyro_rad_per_second;
    const float pitch_correction = pitch_pid_.Update(
        WrapAngle(desired_pitch - pitch_rad_), gyro.y, timestep);
    const float roll_correction = roll_pid_.Update(
        WrapAngle(desired_roll - roll_rad_), gyro.x, timestep);
    const float yaw_correction = yaw_pid_.Update(
        WrapAngle(target_heading_rad_ - yaw_rad_), gyro.z, timestep);
    MixMotorTargets(output, throttle, pitch_correction, roll_correction, yaw_correction);
}

void PositionHoldController::CaptureTarget(const ControllerInput &input) {
    target_north_ft_ = input.gps.position_ned_ft.x;
    target_east_ft_ = input.gps.position_ned_ft.y;
    target_altitude_ft_ = BarometricAltitudeFt(input.barometer);
    target_heading_rad_ = MagneticHeading(input.magnetometer);
    const SensorVector3 &acceleration = input.imu.body_specific_force_ft_per_second_squared;
    const float acceleration_squared = acceleration.x * acceleration.x
        + acceleration.y * acceleration.y + acceleration.z * acceleration.z;
    if (acceleration_squared > MinimumAccelerationSquared) {
        pitch_rad_ = std::atan2(-acceleration.x,
            std::sqrt(acceleration.y * acceleration.y + acceleration.z * acceleration.z));
        roll_rad_ = std::atan2(acceleration.y, -acceleration.z);
    }
    yaw_rad_ = target_heading_rad_;
    target_initialized_ = true;
}

void PositionHoldController::UpdateAttitudeEstimate(const ControllerInput &input) {
    const float timestep = input.timestep_seconds;
    const SensorVector3 &gyro = input.imu.body_gyro_rad_per_second;
    const SensorVector3 &acceleration = input.imu.body_specific_force_ft_per_second_squared;
    pitch_rad_ = WrapAngle(pitch_rad_ + gyro.y * timestep);
    roll_rad_ = WrapAngle(roll_rad_ + gyro.x * timestep);
    yaw_rad_ = WrapAngle(yaw_rad_ + gyro.z * timestep);

    const float acceleration_squared = acceleration.x * acceleration.x
        + acceleration.y * acceleration.y + acceleration.z * acceleration.z;
    if (acceleration_squared > MinimumAccelerationSquared) {
        const float measured_pitch = std::atan2(-acceleration.x,
            std::sqrt(acceleration.y * acceleration.y + acceleration.z * acceleration.z));
        const float measured_roll = std::atan2(acceleration.y, -acceleration.z);
        pitch_rad_ = BlendAngle(pitch_rad_, measured_pitch);
        roll_rad_ = BlendAngle(roll_rad_, measured_roll);
    }
    yaw_rad_ = BlendAngle(yaw_rad_, MagneticHeading(input.magnetometer));
}
