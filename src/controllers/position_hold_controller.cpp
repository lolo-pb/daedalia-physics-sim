#include "controllers/position_hold_controller.hpp"
#include "sensors/ideal_barometer.hpp"

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
constexpr float base_throttle = 0.4f;
constexpr double Pi = 3.14159265358979323846;
constexpr double TwoPi = 2.0 * Pi;

constexpr double tilt_gain = 0.05f;

const vec3d world_up{0, 0, -1}; // in ned
const vec3d body_up{0, 0, -1}; // in body

constexpr PidConfig AltitudePidConfig{
		// TODO needs further tuning
		0.0025f,
		0.001f,
		0.00f,
};

constexpr PidConfig TiltPidConfig{
		0.10f,
		0.0f,
		0.02f,
};

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

double DotProduct(const SensorVector3 &left, const SensorVector3 &right) {
	return left.x * right.x + left.y * right.y + left.z * right.z;
}

} // namespace

PositionHoldController::PositionHoldController()
		: altitude_pid_(AltitudePidConfig),
			pitch_pid_(TiltPidConfig),
			roll_pid_(TiltPidConfig) {}

void PositionHoldController::Reset() {}

void PositionHoldController::Update(const ControllerInput &input, ControllerOutput &output) {
	timestep = input.timestep_seconds;
	if (timestep <= 0) return;

	UpdateAttitudeEstimate(input);
	ApplyCorrections(input, output);
}

void PositionHoldController::UpdateAttitudeEstimate(const ControllerInput &input) {
	const vec3d &gyro = input.imu.body_gyro_rad_per_second;

	const vec3d rotation_vector{
			gyro.x * timestep,
			gyro.y * timestep,
			gyro.z * timestep};

	body_to_ned_ = NormalizeQuaternion(MultiplyQuaternions(
			body_to_ned_, QuaternionFromRotationVector(rotation_vector)));
}

void PositionHoldController::ApplyCorrections(const ControllerInput &input, ControllerOutput &output) {
	const vec3d &gyro = input.imu.body_gyro_rad_per_second;
	const double altitude = BarometricAltitudeFt(input.barometer.pressure_psf);

	const SensorVector3 body_up_in_world = RotateVector(body_to_ned_, body_up);

	// If the drone is inverted
	double vertical_alignment = DotProduct(world_up, body_up_in_world);
	double altitude_correction;

	if (vertical_alignment >= 0.0) {
		flipped = false;
		altitude_correction = base_throttle + altitude_pid_.Update(altitude_target - altitude, gyro.y, timestep);
	} else {
		flipped = true;
		altitude_correction = 0;
	}

	// where world-up currently appears in body axes
	const vec3d target_up_in_body = RotateVector(ConjugateQuaternion(body_to_ned_), world_up);

	// “what body-frame rotation levels the drone?”
	const Quaternion tilt_correction_q = QuaternionFromTwoUnitVectors(body_up, target_up_in_body);
	const vec3d tilt_correction = QuaternionToRotationVector(tilt_correction_q);

	const float pitch_correction = pitch_pid_.Update(tilt_correction.y, gyro.y, timestep);
	const float roll_correction = roll_pid_.Update(tilt_correction.x, gyro.x, timestep);

	MixMotorTargets(output, altitude_correction,
									pitch_correction,
									roll_correction,
									0);
}

/*
we are getting there, we need to change gyro.y from the pid corrections for altitude

and need to work in the initializations and reset.

update pid to just recieve error and timestep
*/
