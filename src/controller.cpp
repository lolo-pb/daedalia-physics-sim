#include "controller.hpp"

void UpdateDemoController(const ControllerInput &input, TargetDrone &drone) {
    static_cast<void>(input);
    drone.SetMotorTarget(MotorId::FrontLeft, 0.8f);
    drone.SetMotorTarget(MotorId::FrontRight, 0.8f);
    drone.SetMotorTarget(MotorId::RearRight, 0.8f);
    drone.SetMotorTarget(MotorId::RearLeft, 0.8f);
}
