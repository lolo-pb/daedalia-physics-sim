#pragma once

#include "controllers/controller_io.hpp"
#include "controllers/pid.hpp"
#include "controllers/quaternion.hpp"

class AngleModeController {
public:
	AngleModeController();

	void Reset();
	void Update(const ControllerInput &input, ControllerOutput &output);
	float GetThrottle() const;

private:
	void UpdateAttitudeEstimate(const ControllerInput &input);
	void ApplyCorrections(const ControllerInput &input, ControllerOutput &output);
	void ApplyKeyboardTiltOffset(const ControllerInput &input);
	void SetWorldEstimate(const ControllerInput &input);

	enum ControllerState {
		WakeUp,
		Initializing,
		Armed,
	};
	ControllerState state = WakeUp;

	Quaternion body_to_ned_;
	Quaternion target_body_to_ned;

	Pid altitude_pid_;
	Pid pitch_pid_;
	Pid roll_pid_;

	float timestep;
	double altitude_target = 10;

	float altitude_correction;
	float pitch_correction;
	float roll_correction;

	bool flipped = false;
};
