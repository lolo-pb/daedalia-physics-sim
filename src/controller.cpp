#include "controller.hpp"

#include "drone.hpp"

void UpdateDemoController(Drone &drone) {
    for (Motor &motor : drone.GetMotors()) {
        motor.thrust_newtons = 3.0f;
        motor.reaction_torque_newton_metres = 0.01f;
    }
}
