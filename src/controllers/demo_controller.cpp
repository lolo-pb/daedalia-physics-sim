#include "controllers/demo_controller.hpp"

void DemoController::Update(const ControllerInput &input, TargetDrone &drone) {
  if (input.keys.x && !x_was_down_) {
    armed_ = !armed_;
  }
  x_was_down_ = input.keys.x;

  const float motor_target = armed_ ? 0.8f : 0.0f;
  drone.SetMotorTarget(MotorId::FrontLeft, motor_target);
  drone.SetMotorTarget(MotorId::FrontRight, motor_target);
  drone.SetMotorTarget(MotorId::RearRight, motor_target);
  drone.SetMotorTarget(MotorId::RearLeft, motor_target);
}
