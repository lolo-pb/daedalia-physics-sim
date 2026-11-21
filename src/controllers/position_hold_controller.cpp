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
constexpr float base_throttle = 0.40675f; // perfect hover btween (0.40673 ; 0.40675)
constexpr double Pi = 3.14159265358979323846;
constexpr double TwoPi = 2.0 * Pi;

constexpr double tilt_gain = 0.05f;

constexpr double PositionTargetRateFtPerSecond = 10.0;
constexpr double AltitudeTargetRateFtPerSecond = 1.0;
constexpr double MaxHorizontalTilt = 0.21255656; // tan(12 degrees)

const vec3d world_up{0, 0, -1}; // in ned
const vec3d body_up{0, 0, -1}; // in body

/*
					Kp			Ki			Kd
Stable baseline	0.0182330	0.0005266	0.0235160
	0.0182330f,
	0.0005266f,
	0.0235160f,

Fast settling	0.0606159	0.0017887	0.0341801
	0.0606159f,
	0.0017887f,
	0.0341801f,
*/

constexpr PidConfig AltitudePidConfig{
		0.0606159f,
		0.0017887f,
		0.0341801f,
		-100.0f,
		100.0f,
};

constexpr PidConfig TiltPidConfig{
		0.10f,
		0.0f,
		0.02f,
};

constexpr PidConfig PositionPidConfig{
		0.025f,
		0.001f,
		0.060f,
		-15.0f,
		15.0f,
};

float KeyAxis(bool positive, bool negative) {
	return positive - negative;
}

float WrapAngle(float angle) { return std::remainder(angle, TwoPi); }

double UpdatePositionAxis(double error, double velocity, float timestep, float &integral) {
	integral = std::clamp(static_cast<float>(integral + error * timestep),
												PositionPidConfig.minimum_integral, PositionPidConfig.maximum_integral);
	return PositionPidConfig.proportional * error + PositionPidConfig.integral * integral - PositionPidConfig.derivative * velocity;
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

double DotProduct(const SensorVector3 &left, const SensorVector3 &right) {
	return left.x * right.x + left.y * right.y + left.z * right.z;
}

} // namespace

PositionHoldController::PositionHoldController()
		: altitude_pid_(AltitudePidConfig),
			pitch_pid_(TiltPidConfig),
			roll_pid_(TiltPidConfig) {}

void PositionHoldController::Reset() {
	state = WakeUp;
	exact_target_.reset();
	altitude_commands_ = {};
	altitude_target = 10;
	altitude_pid_.Reset();
	pitch_pid_.Reset();
	roll_pid_.Reset();
}

void PositionHoldController::Update(const ControllerInput &input, ControllerOutput &output) {
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
	ApplyCorrections(input, output);

	MixMotorTargets(output, altitude_commands_.collective, pitch_correction, roll_correction, 0);
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
	const vec3d &position = input.gps.position_ned_ft;
	const vec3d &velocity = input.gps.velocity_ned_ft_per_second;
	const double altitude = BarometricAltitudeFt(input.barometer.pressure_psf);

	if (exact_target_) {
		north_target_ft_ = exact_target_->north_ft;
		east_target_ft_ = exact_target_->east_ft;
		altitude_target = exact_target_->height_ft;
	} else {
		north_target_ft_ += PositionTargetRateFtPerSecond * KeyAxis(input.keys.w, input.keys.s) * timestep;
		east_target_ft_ += PositionTargetRateFtPerSecond * KeyAxis(input.keys.d, input.keys.a) * timestep;
		altitude_target += AltitudeTargetRateFtPerSecond * KeyAxis(input.keys.r, input.keys.f) * timestep;
	}
	const SensorVector3 body_up_in_world = RotateVector(body_to_ned_, body_up);

	// If the drone is inverted
	double vertical_alignment = DotProduct(world_up, body_up_in_world);
	const double vertical_speed = -input.gps.velocity_ned_ft_per_second.z;

	if (vertical_alignment >= 0.0) {
		flipped = false;
		const float pid = altitude_pid_.Update(altitude_target - altitude, vertical_speed, timestep);
		altitude_commands_ = {base_throttle, pid, base_throttle + pid};
	} else {
		flipped = true;
		altitude_commands_ = {};
	}

	const double north_tilt = UpdatePositionAxis(north_target_ft_ - position.x, velocity.x, timestep, north_integral_);
	const double east_tilt = UpdatePositionAxis(east_target_ft_ - position.y, velocity.y, timestep, east_integral_);
	const double horizontal_scale = MaxHorizontalTilt / std::max(MaxHorizontalTilt, std::hypot(north_tilt, east_tilt));
	const vec3d target_up_in_ned = NormalizeVector({north_tilt * horizontal_scale, east_tilt * horizontal_scale, -1.0});

	// where the GPS-adjusted world-up target appears in body axes
	const vec3d target_up_in_body = RotateVector(ConjugateQuaternion(body_to_ned_), target_up_in_ned);

	// “what body-frame rotation tilts the drone toward the target?”
	const Quaternion tilt_correction_q = QuaternionFromTwoUnitVectors(body_up, target_up_in_body);
	const vec3d tilt_correction = QuaternionToRotationVector(tilt_correction_q);

	pitch_correction = pitch_pid_.Update(tilt_correction.y, gyro.y, timestep);
	roll_correction = roll_pid_.Update(tilt_correction.x, gyro.x, timestep);
}

void PositionHoldController::SetWorldEstimate(const ControllerInput &input) {
	const vec3d &position = input.gps.position_ned_ft;
	north_target_ft_ = position.x;
	east_target_ft_ = position.y;
	north_integral_ = 0.0f;
	east_integral_ = 0.0f;

	const vec3d body_down = -NormalizeVector(input.imu.body_specific_force_ft_per_second_squared);
	body_to_ned_ = QuaternionFromTwoUnitVectors(body_down, -world_up);
	const vec3d field = RotateVector(body_to_ned_, input.magnetometer.body_magnetic_field_microteslas);
	const Quaternion align_north = QuaternionFromRotationVector({0.0, 0.0, -std::atan2(field.y, field.x)});
	body_to_ned_ = NormalizeQuaternion(MultiplyQuaternions(align_north, body_to_ned_));
}

/*
we are getting there, we need to change gyro.y from the pid corrections for altitude

and need to work in the initializations and reset.

update pid to just recieve error and timestep
*/
