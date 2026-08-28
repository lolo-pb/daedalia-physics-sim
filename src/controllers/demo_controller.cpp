#include "controllers/demo_controller.hpp"

void DemoController::Update(
    const ControllerInput &input,
    MotorCommands &motor_commands) {
  if (input.keys.x && !x_was_down_) {
    armed_ = !armed_;
  }
  x_was_down_ = input.keys.x;

  const float motor_target = armed_ ? 0.8f : 0.0f;
  motor_commands.SetMotor(0, motor_target);
  motor_commands.SetMotor(1, motor_target);
  motor_commands.SetMotor(2, motor_target);
  motor_commands.SetMotor(3, motor_target);
}
