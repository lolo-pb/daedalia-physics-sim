#pragma once

#include "controllers/controller_io.hpp"
#include "controllers/pid.hpp"
#include "controllers/quaternion.hpp"

class PositionHoldController {
public:
	PositionHoldController();

	void Reset();
	void Update(const ControllerInput &input, ControllerOutput &output);

private:
	void UpdateAttitudeEstimate(const ControllerInput &input, ControllerOutput &output);

	Quaternion body_to_ned_;
	Quaternion target_body_to_ned;
};
