#include "controllers/tbc_controller.hpp"
#include "sensors/ideal_barometer.hpp"

#include <array>
#include <cmath>

namespace {

//constexpr float HoverThrottle = 0.981f; // This is an estimate
constexpr float HoverThrottle = 0.97f; // This is an estimate
constexpr float TargetAltitude = 2.0f; // target Altitude manm self explanatory

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

constexpr PidConfig AltitudePidConfig{
		0.10f, // proportional
		0.01f, // integral
		0.10f, // derivative
		-2.0f, // minimum integral
		2.0f, // maximum integral
		-0.20f, // minimum output
		0.20f, // maximum output
};

} // namespace

TBCController::TBCController() : altitude_pid_(AltitudePidConfig){};

void TBCController::Reset() {
	orientation_ = {};
	initialized_ = false;
}

void TBCController::Update(const ControllerInput &input, MotorCommands &motor_commands) {
	const float timestep = input.timestep_seconds;
	if (!std::isfinite(timestep) || timestep <= 0.0f) { return; }

	UpdateAttitudeEstimate(input.imu, timestep);

	/// ALTITUDE CONTROL ///

	const float altitude = BarometricAltitudeMeters(input.barometer.pressure_pascals);
	const float error = TargetAltitude - altitude;
	const float lift_correction = altitude_pid_.Update(error, input.gps.world_velocity_meters_per_second.y, timestep);

	float targetThrottle = (lift_correction + 1) * HoverThrottle;

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
			//motor_commands.SetMotor(i, ((HoverThrottle / sumy) * motorWorldNormals[i].y));
			motor_commands.SetMotor(i, ((targetThrottle / sumy) * motorWorldNormals[i].y));
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

/*
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣀⠤
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⡘⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣠⠶⠚⠁⠀
⠀⠀⠂⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢠⡜⠁⠀⠀⠀⠀⠀⠀⠐⢆⡀⠀⠀⠀⠀⢀⣤⣶⠟⠋⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠂⠀⠀⠐⠀⠀⠀⠂⠀⠀⠠⠀⠀⠀⠀⠀⠀⢀⣤⠞⠁⠀⠀⠀⠀⣀⣀⠀⠀⠈⠑⠲⢶⠒⠖⣺⡿⠏⠀⠀⠀⠀⠀⠀
⠀⠀⠂⠀⠀⠀⠀⠀⢀⣀⣀⣀⣠⣤⣤⣤⣤⣤⣤⣤⡤⠴⠚⡏⠀⠀⣀⡤⣖⣛⣏⢩⡟⠽⠭⠃⠶⢤⣮⣡⣾⠋⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠠⠤⠐⠒⠋⠉⠉⠉⠀⠀⠀⠀⠀⠈⠉⠉⠙⠲⢤⣤⣀⣹⡴⢙⡟⠋⠁⠀⠈⠑⠍⠊⠲⢤⣀⡼⠛⠉⠀⠙⢊⣱⣖⢶⠶⠶⠶⠒
⠀⠀⠀⠀⠀⠀⠀⠀⠀⣀⣀⣀⣀⠀⠀⠀⠀⠀⠀⣀⣠⢔⡟⠉⠛⡞⠤⣀⣀⣀⣀⣀⡤⠖⠒⠋⠉⠉⠉⠙⠛⠛⠚⠉⠀⠀⠀⠀⠀⠀
⠀⠀⣀⢤⣴⣒⣾⣉⣛⢋⠉⠭⠾⠯⢯⢝⣹⣟⠛⢳⠛⣟⣒⢲⣞⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
⠴⠛⠉⠉⠀⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠈⠉⣳⣫⡴⠓⠒⠒⠉⠛⠲⢤⣀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣠⣯⢻⠁⠀⠀⠀⠀⠀⠀⠈⠙⠲⢄⡀⠀⠀⠀⠀⢀⠀⠠⠀⠀⠀⡀⠀⠀⠄⠀⠂⠀⠀
⠀⠀⠀⠀⠀⠀⠦⣀⡀⠀⠀⠀⠀⠀⠀⣠⣼⡵⠗⠉⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠈⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
⢀⠀⠂⠀⠠⠀⠀⠀⠈⠉⠓⠒⠶⠚⠙⠃⠉⠀⠀⠀⠀⠀⠀⠀⠀⠠⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠄⠀⠀⠀⠀⠀⠀⠀⠀

*/