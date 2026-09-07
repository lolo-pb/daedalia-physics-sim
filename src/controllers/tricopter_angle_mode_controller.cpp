#include "controllers/tricopter_angle_mode_controller.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace {

constexpr float Pi = 3.14159265358979323846f;
constexpr float TiltRadians = 10.0f * Pi / 180.0f;
constexpr float ThrottleRatePerSecond = 0.25f;
constexpr float HoverThrottle = 0.572f;
constexpr float MinimumAccelerationSquared = 1.0e-6f;
constexpr SensorVector3 WorldUp{0.0f, 1.0f, 0.0f};
constexpr std::size_t FrontLeftMotor = 0;
constexpr std::size_t FrontRightMotor = 1;
constexpr std::size_t RearMotor = 2;

constexpr PidConfig PitchPidConfig{
    1.0f, 0.0f, 0.1f, -1.0f, 1.0f, -0.25f, 0.25f,
};
constexpr PidConfig RollPidConfig = PitchPidConfig;

float KeyAxis(bool positive, bool negative) {
  return static_cast<float>(positive) - static_cast<float>(negative);
}

SensorVector3 ScaleVector(const SensorVector3 &vector, float scale) {
  return {vector.x * scale, vector.y * scale, vector.z * scale};
}

SensorVector3 BuildTargetUp(float pitch, float roll) {
  return {
      std::sin(roll) * std::cos(pitch),
      std::cos(roll) * std::cos(pitch),
      -std::sin(pitch),
  };
}

void MixMotorTargets(MotorCommands &motor_commands, float throttle,
                     float pitch_correction, float roll_correction) {
  motor_commands.SetMotor(FrontLeftMotor,
                          throttle + pitch_correction - roll_correction);
  motor_commands.SetMotor(FrontRightMotor,
                          throttle + pitch_correction + roll_correction);
  motor_commands.SetMotor(RearMotor, throttle - 2.0f * pitch_correction);
}

} // namespace

TricopterAngleModeController::TricopterAngleModeController()
    : pitch_pid_(PitchPidConfig), roll_pid_(RollPidConfig) {}

void TricopterAngleModeController::Reset() {
  pitch_pid_.Reset();
  roll_pid_.Reset();
  orientation_ = {};
  throttle_ = HoverThrottle;
  attitude_initialized_ = false;
}

void TricopterAngleModeController::Update(const ControllerInput &input,
                                          MotorCommands &motor_commands) {
  const float timestep = input.timestep_seconds;
  if (timestep <= 0.0f) {
    return;
  }

  const ControllerKeys &keys = input.keys;
  const float target_pitch_rad = TiltRadians * KeyAxis(keys.s, keys.w);
  const float target_roll_rad = TiltRadians * KeyAxis(keys.a, keys.d);
  throttle_ = std::clamp(throttle_ + ThrottleRatePerSecond *
                         KeyAxis(keys.r, keys.f) * timestep, 0.0f, 1.0f);

  UpdateAttitudeEstimate(input.imu, timestep);

  const SensorVector3 &gyro = input.imu.body_gyro_rad_per_second;
  const SensorVector3 current_up =
      RotateVector(ConjugateQuaternion(orientation_), WorldUp);
  const SensorVector3 target_up =
      BuildTargetUp(target_pitch_rad, target_roll_rad);
  const Quaternion tilt_error =
      QuaternionFromTwoUnitVectors(target_up, current_up);
  const SensorVector3 rotation_error =
      QuaternionToRotationVector(tilt_error);
  const float pitch_correction =
      pitch_pid_.Update(rotation_error.x, gyro.x, timestep);
  const float roll_correction =
      roll_pid_.Update(rotation_error.z, gyro.z, timestep);

  MixMotorTargets(motor_commands, throttle_, pitch_correction,
                  roll_correction);
}

float TricopterAngleModeController::GetThrottle() const { return throttle_; }

void TricopterAngleModeController::UpdateAttitudeEstimate(const ImuSample &imu,
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
