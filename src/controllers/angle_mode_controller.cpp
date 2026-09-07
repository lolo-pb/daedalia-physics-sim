#include "controllers/angle_mode_controller.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace {

constexpr float Pi = 3.14159265358979323846f;
constexpr float TwoPi = 2.0f * Pi;
constexpr float TiltRadians = 10.0f * Pi / 180.0f;
constexpr float YawRateRadiansPerSecond = 45.0f * Pi / 180.0f;
constexpr float ThrottleRatePerSecond = 0.25f;
constexpr float HoverThrottle = 0.70f;
constexpr float MinimumAccelerationSquared = 1.0e-6f;
constexpr SensorVector3 WorldUp{0.0f, 1.0f, 0.0f};
constexpr std::size_t FrontLeftMotor = 0;
constexpr std::size_t FrontRightMotor = 1;
constexpr std::size_t RearRightMotor = 2;
constexpr std::size_t RearLeftMotor = 3;

constexpr PidConfig PitchPidConfig{
    1.0f, 0.0f, 0.1f, -1.0f, 1.0f, -0.25f, 0.25f,
};
constexpr PidConfig RollPidConfig = PitchPidConfig;
constexpr PidConfig YawPidConfig = PitchPidConfig;

float KeyAxis(bool positive, bool negative) {
  return static_cast<float>(positive) - static_cast<float>(negative);
}

float WrapAngle(float angle) { return std::remainder(angle, TwoPi); }

SensorVector3 ScaleVector(const SensorVector3 &vector, float scale) {
  return {vector.x * scale, vector.y * scale, vector.z * scale};
}

Quaternion BuildTargetOrientation(float pitch, float roll, float yaw) {
  const Quaternion pitch_rotation =
      QuaternionFromRotationVector({pitch, 0.0f, 0.0f});
  const Quaternion roll_rotation =
      QuaternionFromRotationVector({0.0f, 0.0f, roll});
  const Quaternion yaw_rotation =
      QuaternionFromRotationVector({0.0f, yaw, 0.0f});
  return MultiplyQuaternions(
      yaw_rotation, MultiplyQuaternions(pitch_rotation, roll_rotation));
}

void MixMotorTargets(MotorCommands &motor_commands, float throttle,
                     float pitch_correction, float roll_correction,
                     float yaw_correction) {
  motor_commands.SetMotor(FrontLeftMotor, throttle + pitch_correction -
                                              roll_correction + yaw_correction);
  motor_commands.SetMotor(FrontRightMotor, throttle + pitch_correction +
                                               roll_correction -
                                               yaw_correction);
  motor_commands.SetMotor(RearRightMotor, throttle - pitch_correction +
                                              roll_correction + yaw_correction);
  motor_commands.SetMotor(RearLeftMotor, throttle - pitch_correction -
                                             roll_correction - yaw_correction);
}

} // namespace

AngleModeController::AngleModeController()
    : pitch_pid_(PitchPidConfig), roll_pid_(RollPidConfig),
      yaw_pid_(YawPidConfig) {}

void AngleModeController::Reset() {
  pitch_pid_.Reset();
  roll_pid_.Reset();
  yaw_pid_.Reset();
  orientation_ = {};
  target_yaw_rad_ = 0.0f;
  throttle_ = HoverThrottle;
  attitude_initialized_ = false;
}

void AngleModeController::Update(const ControllerInput &input,
                                 MotorCommands &motor_commands) {
  const float timestep = input.timestep_seconds;
  if (timestep <= 0.0f) {
    return;
  }

  const ControllerKeys &keys = input.keys;
  const float target_pitch_rad = TiltRadians * KeyAxis(keys.s, keys.w);
  const float target_roll_rad = TiltRadians * KeyAxis(keys.a, keys.d);
  target_yaw_rad_ = WrapAngle(target_yaw_rad_ + YawRateRadiansPerSecond *
                              KeyAxis(keys.q, keys.e) * timestep);
  throttle_ = std::clamp(throttle_ + ThrottleRatePerSecond *
                         KeyAxis(keys.r, keys.f) * timestep, 0.0f, 1.0f);

  UpdateAttitudeEstimate(input.imu, timestep);
  const SensorVector3 &gyro = input.imu.body_gyro_rad_per_second;
  const Quaternion target_orientation = BuildTargetOrientation(
      target_pitch_rad, target_roll_rad, target_yaw_rad_);
  const Quaternion orientation_error = MultiplyQuaternions(
      ConjugateQuaternion(orientation_), target_orientation);
  const SensorVector3 rotation_error =
      QuaternionToRotationVector(orientation_error);
  const float pitch_correction =
      pitch_pid_.Update(rotation_error.x, gyro.x, timestep);
  const float roll_correction =
      roll_pid_.Update(rotation_error.z, gyro.z, timestep);
  const float yaw_correction =
      yaw_pid_.Update(rotation_error.y, gyro.y, timestep);

  MixMotorTargets(motor_commands, throttle_, pitch_correction, roll_correction,
                  yaw_correction);
}

float AngleModeController::GetThrottle() const { return throttle_; }

void AngleModeController::UpdateAttitudeEstimate(const ImuSample &imu,
                                                 float timestep) {
  const SensorVector3 &gyro = imu.body_gyro_rad_per_second;
  const SensorVector3 &acceleration =
      imu.body_specific_force_meters_per_second_squared;
  const float acceleration_squared = acceleration.x * acceleration.x +
                                     acceleration.y * acceleration.y +
                                     acceleration.z * acceleration.z;
  const bool has_accelerometer_attitude = acceleration_squared >
                                          MinimumAccelerationSquared;
  const SensorVector3 measured_up = NormalizeVector(acceleration);

  if (!attitude_initialized_) {
    if (has_accelerometer_attitude) {
      orientation_ = QuaternionFromTwoUnitVectors(measured_up, WorldUp);
    }
    attitude_initialized_ = true;
    return;
  }

  const Quaternion gyro_rotation = QuaternionFromRotationVector(
      ScaleVector(gyro, timestep));
  orientation_ = NormalizeQuaternion(
      MultiplyQuaternions(orientation_, gyro_rotation));
}
