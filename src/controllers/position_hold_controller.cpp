#include "controllers/position_hold_controller.hpp"

#include <algorithm>
#include <cmath>

/*
Position Hold controller:
- After reset or controller entry, it first recovers level from gravity without
  applying position corrections.
- GPS north/east position PID produces a desired north/east velocity.
- GPS velocity PID produces a desired horizontal acceleration.
- That acceleration becomes pitch/roll targets, limited to 10 degrees.
- IMU attitude/rate PID produces the F450's four individual motor commands.
- Barometric altitude PID produces collective throttle; GPS vertical velocity damps it.
*/
namespace {

constexpr float Pi = 3.14159265358979323846f;
constexpr float TwoPi = 2.0f * Pi;
constexpr float GravityFtPerSecondSquared = 32.174f;
constexpr float SeaLevelPressurePsf = 2116.21662f;
constexpr float AtmosphericScaleHeightFt = 27672.2f;
constexpr float MinimumAccelerationSquared = 1.0e-6f;
constexpr float HoverThrottle = 0.35f;
constexpr float HorizontalTargetSpeedFtPerSecond = 4.0f;
constexpr float AltitudeTargetSpeedFtPerSecond = 3.0f;
constexpr float YawTargetRateRadiansPerSecond = 45.0f * Pi / 180.0f;
constexpr float MaximumTiltRadians = 10.0f * Pi / 180.0f;
constexpr float LevelRecoveryToleranceRadians = 3.0f * Pi / 180.0f;
constexpr float LevelRecoveryRateToleranceRadiansPerSecond = 10.0f * Pi / 180.0f;
constexpr int GravityInitializationSamples = 12;
constexpr float MinimumGravitySampleFtPerSecondSquared = 0.8f * GravityFtPerSecondSquared;
constexpr float MaximumGravitySampleFtPerSecondSquared = 1.2f * GravityFtPerSecondSquared;

constexpr PidConfig PositionPidConfig{0.45f, 0.0f, 0.0f, 0.0f, 0.0f, -5.0f, 5.0f};
constexpr PidConfig VelocityPidConfig{0.90f, 0.0f, 0.0f, 0.0f, 0.0f, -5.0f, 5.0f};
constexpr PidConfig AltitudePidConfig{0.025f, 0.002f, 0.08f, -30.0f, 30.0f, -0.30f, 0.30f};
constexpr PidConfig PitchPidConfig{0.60f, 0.0f, 0.12f, -1.0f, 1.0f, -0.12f, 0.12f};
constexpr PidConfig RollPidConfig = PitchPidConfig;
constexpr PidConfig YawPidConfig{0.40f, 0.0f, 0.08f, -1.0f, 1.0f, -0.10f, 0.10f};
constexpr PidConfig EmergencyLevelPidConfig{1.20f, 0.0f, 0.20f, -1.0f, 1.0f, -0.40f, 0.40f};

float KeyAxis(bool positive, bool negative) {
    return static_cast<float>(positive) - static_cast<float>(negative);
}

float WrapAngle(float angle) { return std::remainder(angle, TwoPi); }

float BarometricAltitudeFt(const BarometerSample &barometer) {
    return -AtmosphericScaleHeightFt * std::log(barometer.pressure_psf / SeaLevelPressurePsf);
}

float MagneticHeading(const MagnetometerSample &magnetometer) {
    const SensorVector3 &field = magnetometer.body_magnetic_field_microteslas;
    return std::atan2(field.y, field.x);
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
    : north_position_pid_(PositionPidConfig), east_position_pid_(PositionPidConfig),
      north_velocity_pid_(VelocityPidConfig), east_velocity_pid_(VelocityPidConfig),
      altitude_pid_(AltitudePidConfig), pitch_pid_(PitchPidConfig), roll_pid_(RollPidConfig),
      yaw_pid_(YawPidConfig), emergency_pitch_pid_(EmergencyLevelPidConfig),
      emergency_roll_pid_(EmergencyLevelPidConfig) {}

void PositionHoldController::Reset() {
    north_position_pid_.Reset();
    east_position_pid_.Reset();
    north_velocity_pid_.Reset();
    east_velocity_pid_.Reset();
    altitude_pid_.Reset();
    pitch_pid_.Reset();
    roll_pid_.Reset();
    yaw_pid_.Reset();
    emergency_pitch_pid_.Reset();
    emergency_roll_pid_.Reset();
    target_north_ft_ = 0.0f;
    target_east_ft_ = 0.0f;
    target_altitude_ft_ = 0.0f;
    target_heading_rad_ = 0.0f;
    pitch_rad_ = 0.0f;
    roll_rad_ = 0.0f;
    yaw_rad_ = 0.0f;
    body_to_ned_ = {};
    accelerometer_sample_sum_ = {};
    accelerometer_sample_count_ = 0;
    attitude_initialized_ = false;
    position_hold_initialized_ = false;
}

void PositionHoldController::Update(const ControllerInput &input, ControllerOutput &output) {
    if (input.timestep_seconds <= 0.0f) return;
    if (!attitude_initialized_) {
        // Specific force measures gravity only while its magnitude is near one g.
        MixMotorTargets(output, HoverThrottle, 0.0f, 0.0f, 0.0f);
        CollectAccelerometerInitializationSample(input);
        return;
    }

    if (!position_hold_initialized_) {
        if (RecoverLevel(input, output)) {
            CaptureTarget(input);
            position_hold_initialized_ = true;
        }
        return;
    }

    UpdateAttitudeEstimate(input);

    const float timestep = input.timestep_seconds;
    const ControllerKeys &keys = input.keys;
    target_north_ft_ += HorizontalTargetSpeedFtPerSecond * KeyAxis(keys.w, keys.s) * timestep;
    target_east_ft_ += HorizontalTargetSpeedFtPerSecond * KeyAxis(keys.d, keys.a) * timestep;
    target_altitude_ft_ += AltitudeTargetSpeedFtPerSecond * KeyAxis(keys.r, keys.f) * timestep;
    target_heading_rad_ = WrapAngle(target_heading_rad_
        + YawTargetRateRadiansPerSecond * KeyAxis(keys.q, keys.e) * timestep);

    const GpsSample &gps = input.gps;
    const float desired_north_velocity = north_position_pid_.Update(
        target_north_ft_ - gps.position_ned_ft.x, 0.0f, timestep);
    const float desired_east_velocity = east_position_pid_.Update(
        target_east_ft_ - gps.position_ned_ft.y, 0.0f, timestep);
    const float north_acceleration = north_velocity_pid_.Update(
        desired_north_velocity - gps.velocity_ned_ft_per_second.x, 0.0f, timestep);
    const float east_acceleration = east_velocity_pid_.Update(
        desired_east_velocity - gps.velocity_ned_ft_per_second.y, 0.0f, timestep);

    const float heading_sine = std::sin(yaw_rad_);
    const float heading_cosine = std::cos(yaw_rad_);
    const float forward_acceleration = north_acceleration * heading_cosine + east_acceleration * heading_sine;
    const float right_acceleration = -north_acceleration * heading_sine + east_acceleration * heading_cosine;
    const float desired_pitch = std::clamp(-std::atan2(forward_acceleration, GravityFtPerSecondSquared),
        -MaximumTiltRadians, MaximumTiltRadians);
    const float desired_roll = std::clamp(std::atan2(right_acceleration, GravityFtPerSecondSquared),
        -MaximumTiltRadians, MaximumTiltRadians);

    const float altitude_correction = altitude_pid_.Update(
        target_altitude_ft_ - BarometricAltitudeFt(input.barometer),
        -gps.velocity_ned_ft_per_second.z, timestep);
    const SensorVector3 &gyro = input.imu.body_gyro_rad_per_second;
    const float pitch_correction = pitch_pid_.Update(
        WrapAngle(desired_pitch - pitch_rad_), gyro.y, timestep);
    const float roll_correction = roll_pid_.Update(
        WrapAngle(desired_roll - roll_rad_), gyro.x, timestep);
    const float yaw_correction = yaw_pid_.Update(
        WrapAngle(target_heading_rad_ - yaw_rad_), gyro.z, timestep);
    MixMotorTargets(output, std::clamp(HoverThrottle + altitude_correction, 0.0f, 1.0f),
        pitch_correction, roll_correction, yaw_correction);
}

void PositionHoldController::CaptureTarget(const ControllerInput &input) {
    target_north_ft_ = input.gps.position_ned_ft.x;
    target_east_ft_ = input.gps.position_ned_ft.y;
    target_altitude_ft_ = BarometricAltitudeFt(input.barometer);
    target_heading_rad_ = MagneticHeading(input.magnetometer);
}

void PositionHoldController::CollectAccelerometerInitializationSample(const ControllerInput &input) {
    const SensorVector3 &acceleration = input.imu.body_specific_force_ft_per_second_squared;
    const float acceleration_squared = acceleration.x * acceleration.x
        + acceleration.y * acceleration.y + acceleration.z * acceleration.z;
    if (acceleration_squared <= MinimumAccelerationSquared) return;
    const float acceleration_magnitude = std::sqrt(acceleration_squared);
    if (acceleration_magnitude < MinimumGravitySampleFtPerSecondSquared
        || acceleration_magnitude > MaximumGravitySampleFtPerSecondSquared) return;

    accelerometer_sample_sum_.x += acceleration.x;
    accelerometer_sample_sum_.y += acceleration.y;
    accelerometer_sample_sum_.z += acceleration.z;
    ++accelerometer_sample_count_;
    if (accelerometer_sample_count_ < GravityInitializationSamples) return;

    const double inverse_sample_count = 1.0 / accelerometer_sample_count_;
    const double specific_force_x = accelerometer_sample_sum_.x * inverse_sample_count;
    const double specific_force_y = accelerometer_sample_sum_.y * inverse_sample_count;
    const double specific_force_z = accelerometer_sample_sum_.z * inverse_sample_count;

    // An accelerometer measures specific force, which is opposite gravity at hover.
    pitch_rad_ = std::atan2(specific_force_x,
        std::sqrt(specific_force_y * specific_force_y + specific_force_z * specific_force_z));
    roll_rad_ = std::atan2(-specific_force_y, -specific_force_z);
    target_heading_rad_ = MagneticHeading(input.magnetometer);
    yaw_rad_ = target_heading_rad_;
    body_to_ned_ = NormalizeQuaternion(MultiplyQuaternions(
        MultiplyQuaternions(QuaternionFromRotationVector({0.0, 0.0, yaw_rad_}),
                            QuaternionFromRotationVector({0.0, pitch_rad_, 0.0})),
        QuaternionFromRotationVector({roll_rad_, 0.0, 0.0})));
    attitude_initialized_ = true;
}

bool PositionHoldController::RecoverLevel(const ControllerInput &input, ControllerOutput &output) {
    const SensorVector3 &gyro = input.imu.body_gyro_rad_per_second;
    const float pitch_correction = emergency_pitch_pid_.Update(
        WrapAngle(-pitch_rad_), gyro.y, input.timestep_seconds);
    const float roll_correction = emergency_roll_pid_.Update(
        WrapAngle(-roll_rad_), gyro.x, input.timestep_seconds);
    const float yaw_correction = yaw_pid_.Update(
        WrapAngle(target_heading_rad_ - yaw_rad_), gyro.z, input.timestep_seconds);
    MixMotorTargets(output, HoverThrottle, pitch_correction, roll_correction, yaw_correction);

    UpdateAttitudeEstimate(input);
    return std::abs(pitch_rad_) <= LevelRecoveryToleranceRadians
        && std::abs(roll_rad_) <= LevelRecoveryToleranceRadians
        && std::abs(gyro.x) <= LevelRecoveryRateToleranceRadiansPerSecond
        && std::abs(gyro.y) <= LevelRecoveryRateToleranceRadiansPerSecond;
}

void PositionHoldController::UpdateAttitudeEstimate(const ControllerInput &input) {
    const float timestep = input.timestep_seconds;
    const SensorVector3 &gyro = input.imu.body_gyro_rad_per_second;
    body_to_ned_ = NormalizeQuaternion(MultiplyQuaternions(body_to_ned_,
        QuaternionFromRotationVector({gyro.x * timestep, gyro.y * timestep, gyro.z * timestep})));

    const double roll_sine_cosine = 2.0 * (body_to_ned_.w * body_to_ned_.x
        + body_to_ned_.y * body_to_ned_.z);
    const double roll_cosine_cosine = 1.0 - 2.0 * (body_to_ned_.x * body_to_ned_.x
        + body_to_ned_.y * body_to_ned_.y);
    const double pitch_sine = std::clamp(2.0 * (body_to_ned_.w * body_to_ned_.y
        - body_to_ned_.z * body_to_ned_.x), -1.0, 1.0);
    const double yaw_sine_cosine = 2.0 * (body_to_ned_.w * body_to_ned_.z
        + body_to_ned_.x * body_to_ned_.y);
    const double yaw_cosine_cosine = 1.0 - 2.0 * (body_to_ned_.y * body_to_ned_.y
        + body_to_ned_.z * body_to_ned_.z);
    roll_rad_ = static_cast<float>(std::atan2(roll_sine_cosine, roll_cosine_cosine));
    pitch_rad_ = static_cast<float>(std::asin(pitch_sine));
    yaw_rad_ = static_cast<float>(std::atan2(yaw_sine_cosine, yaw_cosine_cosine));
}
