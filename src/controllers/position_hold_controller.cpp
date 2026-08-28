#include "controllers/position_hold_controller.hpp"

#include <algorithm>
#include <cmath>

#include "sensors/ideal_barometer.hpp"

/*
Position Hold behavior:
- After Reset, the first Update captures current GPS X/Z, barometric altitude,
  and magnetic heading. With no keys held, those targets remain fixed.
- W/S and A/D move the horizontal target at 1 m/s relative to target heading;
  R/F move altitude at 0.5 m/s; Q/E turn target heading at 45 degrees/s.
- Horizontal position error requests a world velocity capped at 2 m/s. Velocity
  error requests acceleration capped at 3 m/s^2, then pitch/roll capped at 15
  degrees. Entry velocity is braked toward the captured point, so it may cross it.
- Barometric altitude error and GPS vertical velocity set throttle from 0.45 to
  0.90 around 0.70 hover throttle.
- Gyro estimates blended with accelerometer and magnetometer measurements feed
  attitude PIDs; their limited corrections are mixed into normalized motor targets.
- A non-positive timestep leaves controller state and motor targets unchanged.
*/
namespace {

constexpr float Pi = 3.14159265358979323846f;
constexpr float TwoPi = 2.0f * Pi;
constexpr float GravityMetersPerSecondSquared = 9.81f;
constexpr float ComplementaryGyroWeight = 0.98f;
constexpr float MinimumAccelerationSquared = 1.0e-6f;
constexpr float HoverThrottle = 0.70f;
constexpr float HorizontalPositionGain = 0.8f;
constexpr float HorizontalVelocityGain = 1.8f;
constexpr float MaximumHorizontalApproachSpeedMetersPerSecond = 2.0f;
constexpr float MaximumHorizontalAcceleration = 3.0f;
constexpr float MaximumTiltRadians = 15.0f * Pi / 180.0f;
constexpr float AltitudeProportionalGain = 0.08f;
constexpr float AltitudeIntegralGain = 0.02f;
constexpr float AltitudeVelocityGain = 0.06f;
constexpr float AltitudeIntegralLimit = 2.0f;
constexpr float AttitudeIntegralLimit = 1.0f;
constexpr float AttitudeCorrectionLimit = 0.25f;
constexpr float MinimumThrottle = 0.45f;
constexpr float MaximumThrottle = 0.90f;
constexpr float HorizontalTargetSpeedMetersPerSecond = 1.0f;
constexpr float AltitudeTargetSpeedMetersPerSecond = 0.5f;
constexpr float YawTargetRateRadiansPerSecond = 45.0f * Pi / 180.0f;
constexpr std::size_t FrontLeftMotor = 0;
constexpr std::size_t FrontRightMotor = 1;
constexpr std::size_t RearRightMotor = 2;
constexpr std::size_t RearLeftMotor = 3;

struct PidGains {
    float proportional;
    float integral;
    float derivative;
};

constexpr PidGains PitchGains{1.0f, 0.0f, 0.1f};
constexpr PidGains RollGains{1.0f, 0.0f, 0.1f};
constexpr PidGains YawGains{1.0f, 0.0f, 0.1f};

float KeyAxis(bool positive, bool negative) {
    return static_cast<float>(positive) - static_cast<float>(negative);
}

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

float UpdatePid(
    float error,
    float measured_rate,
    float timestep,
    const PidGains &gains,
    float &integral) {
    integral = std::clamp(
        integral + error * timestep,
        -AttitudeIntegralLimit,
        AttitudeIntegralLimit);
    return std::clamp(
        gains.proportional * error
            + gains.integral * integral
            - gains.derivative * measured_rate,
        -AttitudeCorrectionLimit,
        AttitudeCorrectionLimit);
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

void PositionHoldController::Reset() {
    target_world_x_meters_ = 0.0f;
    target_world_z_meters_ = 0.0f;
    target_altitude_meters_ = 0.0f;
    target_heading_rad_ = 0.0f;
    pitch_rad_ = 0.0f;
    roll_rad_ = 0.0f;
    yaw_rad_ = 0.0f;
    pitch_integral_ = 0.0f;
    roll_integral_ = 0.0f;
    yaw_integral_ = 0.0f;
    altitude_integral_ = 0.0f;
    target_initialized_ = false;
}

void PositionHoldController::Update(
    const ControllerInput &input,
    MotorCommands &motor_commands) {
    const float timestep = input.timestep_seconds;
    if (timestep <= 0.0f) {
        return;
    }
    if (!target_initialized_) {
        CaptureTarget(input);
    } else {
        UpdateAttitudeEstimate(input);
    }

    const ControllerKeys &keys = input.keys;
    const float forward_input = KeyAxis(keys.w, keys.s);
    const float right_input = KeyAxis(keys.d, keys.a);
    const float heading_sine = std::sin(target_heading_rad_);
    const float heading_cosine = std::cos(target_heading_rad_);
    target_world_x_meters_ += HorizontalTargetSpeedMetersPerSecond
        * (-forward_input * heading_sine + right_input * heading_cosine)
        * timestep;
    target_world_z_meters_ += HorizontalTargetSpeedMetersPerSecond
        * (-forward_input * heading_cosine - right_input * heading_sine)
        * timestep;
    target_altitude_meters_ += AltitudeTargetSpeedMetersPerSecond
        * KeyAxis(keys.r, keys.f)
        * timestep;
    target_heading_rad_ = WrapAngle(
        target_heading_rad_
        + YawTargetRateRadiansPerSecond * KeyAxis(keys.q, keys.e) * timestep);

    const GpsSample &gps = input.gps;
    float desired_world_velocity_x = HorizontalPositionGain
        / HorizontalVelocityGain
        * (target_world_x_meters_ - gps.world_position_meters.x);
    float desired_world_velocity_z = HorizontalPositionGain
        / HorizontalVelocityGain
        * (target_world_z_meters_ - gps.world_position_meters.z);
    const float desired_horizontal_speed = std::hypot(
        desired_world_velocity_x, desired_world_velocity_z);
    if (desired_horizontal_speed
        > MaximumHorizontalApproachSpeedMetersPerSecond) {
        const float scale = MaximumHorizontalApproachSpeedMetersPerSecond
            / desired_horizontal_speed;
        desired_world_velocity_x *= scale;
        desired_world_velocity_z *= scale;
    }

    float world_acceleration_x = HorizontalVelocityGain
        * (desired_world_velocity_x - gps.world_velocity_meters_per_second.x);
    float world_acceleration_z = HorizontalVelocityGain
        * (desired_world_velocity_z - gps.world_velocity_meters_per_second.z);
    const float horizontal_acceleration = std::hypot(
        world_acceleration_x, world_acceleration_z);
    if (horizontal_acceleration > MaximumHorizontalAcceleration) {
        const float scale = MaximumHorizontalAcceleration / horizontal_acceleration;
        world_acceleration_x *= scale;
        world_acceleration_z *= scale;
    }

    const float right_acceleration =
        world_acceleration_x * heading_cosine
        - world_acceleration_z * heading_sine;
    const float forward_acceleration =
        -world_acceleration_x * heading_sine
        - world_acceleration_z * heading_cosine;
    const float desired_pitch = std::clamp(
        -std::atan2(forward_acceleration, GravityMetersPerSecondSquared),
        -MaximumTiltRadians,
        MaximumTiltRadians);
    const float desired_roll = std::clamp(
        -std::atan2(right_acceleration, GravityMetersPerSecondSquared),
        -MaximumTiltRadians,
        MaximumTiltRadians);

    const float altitude =
        BarometricAltitudeMeters(input.barometer.pressure_pascals);
    const float altitude_error = target_altitude_meters_ - altitude;
    altitude_integral_ = std::clamp(
        altitude_integral_ + altitude_error * timestep,
        -AltitudeIntegralLimit,
        AltitudeIntegralLimit);
    const float throttle = std::clamp(
        HoverThrottle
            + AltitudeProportionalGain * altitude_error
            + AltitudeIntegralGain * altitude_integral_
            - AltitudeVelocityGain * gps.world_velocity_meters_per_second.y,
        MinimumThrottle,
        MaximumThrottle);

    const SensorVector3 &gyro = input.imu.body_gyro_rad_per_second;
    const float pitch_correction = UpdatePid(
        WrapAngle(desired_pitch - pitch_rad_),
        gyro.x,
        timestep,
        PitchGains,
        pitch_integral_);
    const float roll_correction = UpdatePid(
        WrapAngle(desired_roll - roll_rad_),
        gyro.z,
        timestep,
        RollGains,
        roll_integral_);
    const float yaw_correction = UpdatePid(
        WrapAngle(target_heading_rad_ - yaw_rad_),
        gyro.y,
        timestep,
        YawGains,
        yaw_integral_);
    MixMotorTargets(
        motor_commands,
        throttle,
        pitch_correction,
        roll_correction,
        yaw_correction);
}

void PositionHoldController::CaptureTarget(const ControllerInput &input) {
    target_world_x_meters_ = input.gps.world_position_meters.x;
    target_world_z_meters_ = input.gps.world_position_meters.z;
    target_altitude_meters_ =
        BarometricAltitudeMeters(input.barometer.pressure_pascals);
    target_heading_rad_ = MagneticHeading(input.magnetometer);

    const SensorVector3 &acceleration =
        input.imu.body_specific_force_meters_per_second_squared;
    const float acceleration_squared = acceleration.x * acceleration.x
        + acceleration.y * acceleration.y
        + acceleration.z * acceleration.z;
    if (acceleration_squared > MinimumAccelerationSquared) {
        pitch_rad_ = std::atan2(-acceleration.z, acceleration.y);
        roll_rad_ = std::atan2(acceleration.x, acceleration.y);
    }
    yaw_rad_ = target_heading_rad_;
    target_initialized_ = true;
}

void PositionHoldController::UpdateAttitudeEstimate(
    const ControllerInput &input) {
    const float timestep = input.timestep_seconds;
    const SensorVector3 &gyro = input.imu.body_gyro_rad_per_second;
    const SensorVector3 &acceleration =
        input.imu.body_specific_force_meters_per_second_squared;
    pitch_rad_ = WrapAngle(pitch_rad_ + gyro.x * timestep);
    roll_rad_ = WrapAngle(roll_rad_ + gyro.z * timestep);
    yaw_rad_ = WrapAngle(yaw_rad_ + gyro.y * timestep);

    const float acceleration_squared = acceleration.x * acceleration.x
        + acceleration.y * acceleration.y
        + acceleration.z * acceleration.z;
    if (acceleration_squared > MinimumAccelerationSquared) {
        pitch_rad_ = BlendAngle(
            pitch_rad_, std::atan2(-acceleration.z, acceleration.y));
        roll_rad_ = BlendAngle(
            roll_rad_, std::atan2(acceleration.x, acceleration.y));
    }
    yaw_rad_ = BlendAngle(yaw_rad_, MagneticHeading(input.magnetometer));
}
