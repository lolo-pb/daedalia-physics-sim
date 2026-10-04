#pragma once

enum class FlightController {
    Demo,
    AngleMode,
    HorizonMode,
    PositionHold,
    TricopterAngleMode,
    TBC,
};

inline bool SelectControllerSlot(int slot, FlightController &active_controller) {
    switch (slot) {
    case 1:
        active_controller = FlightController::Demo;
        return true;
    case 2:
        active_controller = FlightController::AngleMode;
        return true;
    case 3:
        active_controller = FlightController::HorizonMode;
        return true;
    case 4:
        active_controller = FlightController::PositionHold;
        return true;
    case 5:
        active_controller = FlightController::TricopterAngleMode;
        return true;
    case 6:
        active_controller = FlightController::TBC;
        return true;
    default:
        return false;
    }
}
