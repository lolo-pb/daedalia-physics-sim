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
