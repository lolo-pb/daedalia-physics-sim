#pragma once

enum class FlightController {
    Demo,
    ManualHover,
};

inline bool SelectControllerSlot(int slot, FlightController &active_controller) {
    switch (slot) {
    case 1:
        active_controller = FlightController::Demo;
        return true;
    case 2:
        active_controller = FlightController::ManualHover;
        return true;
    default:
        return false;
    }
}
