#include "controllers/tbc_controller.hpp"

namespace {
constexpr SensorVector3 WorldUp{0.0f, 1.0f, 0.0f};
}

TBCController::TBCController() = default;

void TBCController::Reset() {}

void TBCController::Update(const ControllerInput &input, MotorCommands &motor_commands) {
	const float timestep = input.timestep_seconds;

	UpdateAttitudeEstimate(input.imu, timestep);
}

void TBCController::UpdateAttitudeEstimate(const ImuSample &imu, float timestep) {
	const SensorVector3 &gyro = imu.body_gyro_rad_per_second;
	const SensorVector3 &accel = imu.body_specific_force_meters_per_second_squared;

	if (!initialized_) { // Accel is weird and proly needs update to be more realistic
		const SensorVector3 measured_up = NormalizeVector(accel);
		orientation_ = QuaternionFromTwoUnitVectors(measured_up, WorldUp);

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
