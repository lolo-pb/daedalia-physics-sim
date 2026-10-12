#include "controllers/demo_controller.hpp"

void DemoController::Reset() {
	armed_ = false;
	x_was_down_ = false;
}

void DemoController::Update(const ControllerInput &input, ControllerOutput &output) {
	if (input.keys.x && !x_was_down_) { armed_ = !armed_; }
	x_was_down_ = input.keys.x;

	const float motor_target = armed_ ? 0.8f : 0.0f;
	output.SetMotor(0, motor_target);
	output.SetMotor(1, motor_target);
	output.SetMotor(2, motor_target);
	output.SetMotor(3, motor_target);
}
