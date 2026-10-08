#include "controllers/tbc_controller.hpp"

#include <array>
#include <cmath>

namespace {

constexpr float HoverThrottle = 0.981f; // TODO this needs checking

constexpr SensorVector3 WorldUp{0.0f, 1.0f, 0.0f};
constexpr vec3f MotorLocalNormals[6] = {
		{0.4714, 0.8165, 0.3333}, // Upper face 1
		{-0.9426, 0.0000, 0.3338}, // Upper face 2
		{0.4714, -0.8165, 0.3333}, // Upper face 3
		{0.4714, 0.8165, -0.3333}, // Lower face 1
		{-0.9426, 0.0000, -0.3338}, // Lower face 2
		{0.4714, -0.8165, -0.3333} // Lower face 3
};
vec3f motorWorldNormals[6]{};

float motorThrusts[6]{};

} // namespace

TBCController::TBCController() = default;

void TBCController::Reset() {
	orientation_ = {};
	initialized_ = false;
}

void TBCController::Update(const ControllerInput &input, MotorCommands &motor_commands) {
	const float timestep = input.timestep_seconds;
	if (!std::isfinite(timestep) || timestep <= 0.0f) { return; }

	UpdateAttitudeEstimate(input.imu, timestep);

	/// THRUST UP /////

	float sumy = 0;

	// We asign a thrust based on huw much vertically that motro norlam contributes
	for (int i = 0; i < 6; i++) {
		motorWorldNormals[i] = RotateVector(orientation_, MotorLocalNormals[i]);
		float yComponent = motorWorldNormals[i].y;
		sumy += (yComponent > 0) ? yComponent * yComponent : 0;
	}
	for (int i = 0; i < 6; i++) {
		if (motorWorldNormals[i].y > 0 && sumy != 0) {
			//motor_commands.SetMotor(i, (motorWorldNormals[i].y / sumy) * HoverThrottle);
			motor_commands.SetMotor(i, ((HoverThrottle / sumy) * motorWorldNormals[i].y));
		}
	}

	/// THRUST UP /////
}

void TBCController::UpdateAttitudeEstimate(const ImuSample &imu, float timestep) {
	const SensorVector3 &gyro = imu.body_gyro_rad_per_second;
	const SensorVector3 &accel = imu.body_specific_force_meters_per_second_squared;

	if (!initialized_) {
		if (accel.y > 0.0f) {
			const SensorVector3 measured_up = NormalizeVector(accel);
			orientation_ = QuaternionFromTwoUnitVectors(measured_up, WorldUp);
		}

		initialized_ = true;
		return;
	}

	const Quaternion gyro_rotation = QuaternionFromRotationVector({
			gyro.x * timestep,
			gyro.y * timestep,
			gyro.z * timestep,
	});

	orientation_ = NormalizeQuaternion(MultiplyQuaternions(orientation_, gyro_rotation));
}

/* // lolo section //

aparently the projection formula is

 ( (A.B)/(|B|^2) ) * B

 a, B vectors
 y A.B es la proyeccion por |B|
 y el versor de B es B/|B|
 entonces

 (A.B)/|B| por el versor de B



GPT quiere hacer:
You chose:

motor_target_i = scale × p_i

where `p_i` is how upward-facing that motor is.
But that motor’s thrust is tilted, so only a fraction `p_i` of its thrust goes upward:

vertical_lift_i = motor_target_i × p_i

Substitute your motor target rule:

vertical_lift_i = ( (scale × p_i) × p_i ) = ( scale × p_i² )

*/
