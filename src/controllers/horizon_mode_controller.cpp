#include "controllers/horizon_mode_controller.hpp"
#include "sensors/ideal_barometer.hpp"

#include <algorithm>
#include <cmath>

namespace {
constexpr float base_throttle = 0.40675f;
constexpr double Pi = 3.14159265358979323846;
constexpr double TwoPi = 2.0 * Pi;
constexpr double KeyboardTiltRadians = 5.0 * Pi / 180.0;
constexpr double AltitudeTargetRateFtPerSecond = 5.0;

constexpr double tilt_gain = 0.05f;

const vec3d world_up{0, 0, -1};
const vec3d body_up{0, 0, -1};

constexpr PidConfig AltitudePidConfig{
		0.0002f,
		0.000002f,
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
	output.SetMotor(0, throttle + pitch_correction - roll_correction + yaw_correction);
	output.SetMotor(1, throttle - pitch_correction + roll_correction + yaw_correction);
	output.SetMotor(2, throttle + pitch_correction + roll_correction - yaw_correction);
	output.SetMotor(3, throttle - pitch_correction - roll_correction - yaw_correction);
}

double DotProduct(const SensorVector3 &left, const SensorVector3 &right) {
	return left.x * right.x + left.y * right.y + left.z * right.z;
}

} // namespace

HorizonModeController::HorizonModeController()
		: altitude_pid_(AltitudePidConfig),
			pitch_pid_(TiltPidConfig),
			roll_pid_(TiltPidConfig) {}

void HorizonModeController::Reset() {
	state = WakeUp;
}

void HorizonModeController::Update(const ControllerInput &input, ControllerOutput &output) {
	timestep = input.timestep_seconds;
	if (timestep <= 0) return;

	if (state == WakeUp) {
		MixMotorTargets(output, 0, 0, 0, 0);
		state = Initializing;
		return;
	} else if (state == Initializing) {
		SetWorldEstimate(input);
		state = Armed;
	}

	UpdateAttitudeEstimate(input);
	keypressed = input.keys.w || input.keys.a || input.keys.s || input.keys.d;
	ApplyCorrections(input, output, keypressed);
	ApplyKeyboardTiltOffset(input);

	MixMotorTargets(output, altitude_correction, pitch_correction, roll_correction,
									yaw_correction);
}

float HorizonModeController::GetThrottle() const { return base_throttle; }

void HorizonModeController::UpdateAttitudeEstimate(const ControllerInput &input) {
	const vec3d &gyro = input.imu.body_gyro_rad_per_second;

	const vec3d rotation_vector{
			gyro.x * timestep,
			gyro.y * timestep,
			gyro.z * timestep};

	body_to_ned_ = NormalizeQuaternion(MultiplyQuaternions(
			body_to_ned_, QuaternionFromRotationVector(rotation_vector)));
}

void HorizonModeController::ApplyCorrections(const ControllerInput &input,
																						 ControllerOutput &output, bool keypressed) {
	const vec3d &gyro = input.imu.body_gyro_rad_per_second;
	const double altitude = BarometricAltitudeFt(input.barometer.pressure_psf);

	const SensorVector3 body_up_in_world = RotateVector(body_to_ned_, body_up);

	vertical_alignment = DotProduct(world_up, body_up_in_world);

	if (vertical_alignment >= 0.0) {
		flipped = false;
		altitude_correction = base_throttle + altitude_pid_.Update(altitude_target - altitude, gyro.y, timestep);
	} else {
		flipped = true;
		altitude_correction = 0;
	}
	if (keypressed) {
		pitch_correction = 0;
		roll_correction = 0;
		yaw_correction = 0;
		return;
	}

	const vec3d target_up_in_body = RotateVector(ConjugateQuaternion(body_to_ned_), world_up);
	const Quaternion tilt_correction_q = QuaternionFromTwoUnitVectors(body_up, target_up_in_body);
	const vec3d tilt_correction = QuaternionToRotationVector(tilt_correction_q);

	pitch_correction = pitch_pid_.Update(tilt_correction.y, gyro.y, timestep);
	roll_correction = roll_pid_.Update(tilt_correction.x, gyro.x, timestep);
	yaw_correction = 0;
}

void HorizonModeController::ApplyKeyboardTiltOffset(const ControllerInput &input) {
	altitude_target += AltitudeTargetRateFtPerSecond *
										 KeyAxis(input.keys.r, input.keys.f) * timestep;

	const double pitch_tilt = KeyboardTiltRadians * KeyAxis(input.keys.s, input.keys.w);
	const double roll_tilt = KeyboardTiltRadians * KeyAxis(input.keys.d, input.keys.a);

	pitch_correction += pitch_tilt;
	roll_correction += roll_tilt;
	yaw_correction += KeyboardTiltRadians * KeyAxis(input.keys.e, input.keys.q);

	if (vertical_alignment > 0.0) {
		altitude_correction /= vertical_alignment;
	}
}

void HorizonModeController::SetWorldEstimate(const ControllerInput &input) {
	const vec3d body_down = -NormalizeVector(input.imu.body_specific_force_ft_per_second_squared);
	body_to_ned_ = QuaternionFromTwoUnitVectors(body_down, -world_up);
}
