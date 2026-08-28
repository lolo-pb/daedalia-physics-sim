#include "controllers/attitude_controller.hpp"

#include <algorithm>
#include <cmath>

namespace {

constexpr float Pi = 3.14159265358979323846f;
constexpr float TwoPi = 2.0f * Pi;
constexpr float ComplementaryGyroWeight = 0.98f;
constexpr float MinimumAccelerationSquared = 1.0e-6f;
constexpr float IntegralLimit = 1.0f;
constexpr float CorrectionLimit = 0.25f;
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

float WrapAngle(float angle) { return std::remainder(angle, TwoPi); }

float BlendAngle(float gyro_angle, float accelerometer_angle) {
  const float correction = WrapAngle(accelerometer_angle - gyro_angle);
  return WrapAngle(gyro_angle + (1.0f - ComplementaryGyroWeight) * correction);
}

float UpdatePid(float error, float measured_rate, float timestep,
                const PidGains &gains, float &integral) {
  integral =
      std::clamp(integral + error * timestep, -IntegralLimit, IntegralLimit);
  return std::clamp(gains.proportional * error + gains.integral * integral -
                        gains.derivative * measured_rate,
                    -CorrectionLimit, CorrectionLimit);
}

void MixMotorTargets(MotorCommands &motor_commands, float throttle,
                     float pitch_correction, float roll_correction,
                     float yaw_correction) {
  motor_commands.SetMotor(FrontLeftMotor,
                          throttle + pitch_correction - roll_correction +
                              yaw_correction);
  motor_commands.SetMotor(FrontRightMotor,
                          throttle + pitch_correction + roll_correction -
                              yaw_correction);
  motor_commands.SetMotor(RearRightMotor,
                          throttle - pitch_correction + roll_correction +
                              yaw_correction);
  motor_commands.SetMotor(RearLeftMotor,
                          throttle - pitch_correction - roll_correction -
                              yaw_correction);
}

} // namespace

void AttitudeController::Reset() {
  pitch_rad_ = 0.0f;
  roll_rad_ = 0.0f;
  yaw_rad_ = 0.0f;
  pitch_integral_ = 0.0f;
  roll_integral_ = 0.0f;
  yaw_integral_ = 0.0f;
  attitude_initialized_ = false;
}

void AttitudeController::Update(const ControllerInput &input,
                                const AttitudeSetpoint &setpoint,
                                MotorCommands &motor_commands) {
  const float timestep = input.timestep_seconds;
  if (timestep <= 0.0f) {
    return;
  }

  const SensorVector3 &gyro = input.imu.body_gyro_rad_per_second;
  UpdateAttitudeEstimate(input.imu, timestep);

  const float pitch_correction =
      UpdatePid(WrapAngle(setpoint.pitch_rad - pitch_rad_), gyro.x, timestep,
                PitchGains, pitch_integral_);
  const float roll_correction =
      UpdatePid(WrapAngle(setpoint.roll_rad - roll_rad_), gyro.z, timestep,
                RollGains, roll_integral_);
  const float yaw_correction =
      UpdatePid(WrapAngle(setpoint.yaw_rad - yaw_rad_), gyro.y, timestep,
                YawGains, yaw_integral_);

  MixMotorTargets(motor_commands, setpoint.throttle, pitch_correction,
                  roll_correction, yaw_correction);
}

void AttitudeController::UpdateAttitudeEstimate(const ImuSample &imu,
                                                 float timestep) {
  const SensorVector3 &gyro = imu.body_gyro_rad_per_second;
  const SensorVector3 &acceleration =
      imu.body_specific_force_meters_per_second_squared;
  const float acceleration_squared = acceleration.x * acceleration.x +
                                     acceleration.y * acceleration.y +
                                     acceleration.z * acceleration.z;

  const bool has_accelerometer_attitude =
      acceleration_squared > MinimumAccelerationSquared;
  const float accelerometer_pitch = std::atan2(-acceleration.z, acceleration.y);
  const float accelerometer_roll = std::atan2(acceleration.x, acceleration.y);

  if (!attitude_initialized_) {
    if (has_accelerometer_attitude) {
      pitch_rad_ = accelerometer_pitch;
      roll_rad_ = accelerometer_roll;
    }
    attitude_initialized_ = true;
  } else {
    pitch_rad_ = WrapAngle(pitch_rad_ + gyro.x * timestep);
    roll_rad_ = WrapAngle(roll_rad_ + gyro.z * timestep);
    yaw_rad_ = WrapAngle(yaw_rad_ + gyro.y * timestep);

    if (has_accelerometer_attitude) {
      pitch_rad_ = BlendAngle(pitch_rad_, accelerometer_pitch);
      roll_rad_ = BlendAngle(roll_rad_, accelerometer_roll);
    }
  }
}
