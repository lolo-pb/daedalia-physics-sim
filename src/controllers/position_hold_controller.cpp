#include "controllers/position_hold_controller.hpp"

#include <algorithm>
#include <cmath>

/*
Position Hold controller:
- On startup and reset it need to regain the up direction
- first well do attitude tracking and controll
- then altitude correction
- then position correction
- then sensor fusion adn robustness
*/
namespace {

float KeyAxis(bool positive, bool negative) {
	return static_cast<float>(positive) - static_cast<float>(negative);
}

float WrapAngle(float angle) { return std::remainder(angle, TwoPi); }

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

PositionHoldController::PositionHoldController() {}

void PositionHoldController::Reset() {}

void PositionHoldController::Update(const ControllerInput &input, ControllerOutput &output) {
	UpdateAttitudeEstimate(input);
}

void PositionHoldController::UpdateAttitudeEstimate(const ControllerInput &input) {
	const float timestep = input.timestep_seconds;
	const SensorVector3 &gyro = input.imu.body_gyro_rad_per_second;

	const vec3f rotation_vector{
			gyro.x * timestep,
			gyro.y * timestep,
			gyro.z * timestep};

	Quaternion body_to_ned_ = NormalizeQuaternion(MultiplyQuaternions(
			body_to_ned_, QuaternionFromRotationVector(rotation_vector)));

	const Quaternion correction = MultiplyQuaternions(target_body_to_ned, ConjugateQuaternion(body_to_ned_));
}
