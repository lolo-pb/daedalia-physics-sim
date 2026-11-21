#pragma once

#include "controllers/controller_io.hpp"
#include "controllers/pid.hpp"
#include "controllers/quaternion.hpp"

#include "controllers/position_target.hpp"
#include <optional>

struct NnAltitudeCommands {
	float base = 0, correction = 0, collective = 0;
};

class NnAltitudeController {
public:
	NnAltitudeController();

	void Reset();
	// Absolute sensor coordinates: GPS north/east and barometric altitude.
	void SetTarget(PositionTarget target) { exact_target_ = target; }
	NnAltitudeCommands GetNnAltitudeCommands() const { return altitude_commands_; }
	void Update(const ControllerInput &input, ControllerOutput &output);

private:
	void UpdateAttitudeEstimate(const ControllerInput &input);
	void ApplyCorrections(const ControllerInput &input, ControllerOutput &output);
	void SetWorldEstimate(const ControllerInput &input);

	enum ControllerState {
		WakeUp,
		Initializing,
		Armed,
	};
	ControllerState state = WakeUp;

	Quaternion body_to_ned_;
	Quaternion target_body_to_ned;

	Pid pitch_pid_;
	Pid roll_pid_;

	float timestep;
	double altitude_target = 10;
	float north_target_ft_ = 0.0f;
	float east_target_ft_ = 0.0f;
	float north_integral_ = 0.0f;
	float east_integral_ = 0.0f;

	std::optional<PositionTarget> exact_target_;
	NnAltitudeCommands altitude_commands_;
	float pitch_correction;
	float roll_correction;

	bool flipped = false;
};
