#pragma once

#include "controllers/controller_io.hpp"
#include "controllers/pid.hpp"
#include "controllers/quaternion.hpp"

class TBCController {
public:
	TBCController();
	void Reset();
	void Update(const ControllerInput &input, MotorCommands &motor_commands);

private:
	Quaternion orientation_;
	bool initialized_ = false;

	void UpdateAttitudeEstimate(const ImuSample &imu, float timestep);
};
