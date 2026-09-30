#include "controllers/tbc_controller.hpp"

#include <cmath>

namespace {
constexpr SensorVector3 WorldUp{0.0f, 1.0f, 0.0f};
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