#include "controller.hpp"

#include "drone.hpp"

void UpdateDemoController(Drone &drone) {
    drone.SetMotorCommand(MotorId::FrontLeft, 0.8f);
    drone.SetMotorCommand(MotorId::FrontRight, 0.8f);
    drone.SetMotorCommand(MotorId::RearRight, 0.8f);
    drone.SetMotorCommand(MotorId::RearLeft, 0.8f);
}
