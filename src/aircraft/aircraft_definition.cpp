#include "aircraft/aircraft_definition.hpp"

#include <array>
#include <stdexcept>

namespace {

constexpr std::array Controllers{ControllerType::Manual};
constexpr std::array<std::string_view, 7> Controls{
    "fcs/throttle-cmd-norm", "fcs/aileron-cmd-norm", "fcs/elevator-cmd-norm",
    "fcs/rudder-cmd-norm", "fcs/flap-cmd-norm",
    "fcs/left-brake-cmd-norm", "fcs/right-brake-cmd-norm",
};
constexpr std::array<std::string_view, 4> F450Motors{
    "fcs/cmdFR_ext_nd", "fcs/cmdAL_ext_nd", "fcs/cmdFL_ext_nd", "fcs/cmdAR_ext_nd",
};
constexpr std::array<std::string_view, 1> F450DisabledSimulatorControls{
    "fcs/ScasEngage",
};
constexpr std::array<std::string_view, 4> C172DisabledSimulatorControls{
    "ap/attitude_hold", "ap/altitude_hold", "ap/heading_hold", "ap/airspeed_hold",
};
constexpr std::array<std::string_view, 0> NoMotors{};

constexpr std::array<AircraftVisualPart, 9> F450Parts{{
    {{0,0,0}, {0.4f,0.16f,0.5f}, {0.85f,0.35f,0.15f}},
    {{-0.26f,0,-0.26f}, {0.12f,0.08f,0.74f}, {0.3f,0.3f,0.35f}, VisualPartMotion::Fixed, {}, {1,0,0}, 0, 0.785398f},
    {{0.26f,0,-0.26f}, {0.12f,0.08f,0.74f}, {0.3f,0.3f,0.35f}, VisualPartMotion::Fixed, {}, {1,0,0}, 0, -0.785398f},
    {{-0.26f,0,0.26f}, {0.12f,0.08f,0.74f}, {0.3f,0.3f,0.35f}, VisualPartMotion::Fixed, {}, {1,0,0}, 0, -0.785398f},
    {{0.26f,0,0.26f}, {0.12f,0.08f,0.74f}, {0.3f,0.3f,0.35f}, VisualPartMotion::Fixed, {}, {1,0,0}, 0, 0.785398f},
    {{-0.52f,0.12f,-0.52f}, {0.8f,0.025f,0.08f}, {0.8f,0.15f,0.15f}, VisualPartMotion::Rotor, {-0.52f,0.12f,-0.52f}, {0,1,0}, 2},
    {{0.52f,0.12f,-0.52f}, {0.8f,0.025f,0.08f}, {0.8f,0.15f,0.15f}, VisualPartMotion::Rotor, {0.52f,0.12f,-0.52f}, {0,1,0}, 0},
    {{-0.52f,0.12f,0.52f}, {0.8f,0.025f,0.08f}, {0.15f,0.3f,0.8f}, VisualPartMotion::Rotor, {-0.52f,0.12f,0.52f}, {0,1,0}, 1},
    {{0.52f,0.12f,0.52f}, {0.8f,0.025f,0.08f}, {0.15f,0.3f,0.8f}, VisualPartMotion::Rotor, {0.52f,0.12f,0.52f}, {0,1,0}, 3},
}};

constexpr std::array<AircraftVisualPart, 14> C172Parts{{
    {{0,0,0}, {3,3,18}, {0.8f,0.8f,0.85f}},
    {{0,1.5f,-1}, {36,0.25f,4}, {0.85f,0.85f,0.9f}},
    {{0,0.5f,8}, {11,0.2f,2}, {0.8f,0.8f,0.85f}},
    {{0,2,8}, {0.2f,3,3}, {0.2f,0.35f,0.75f}},
    {{-13,1.5f,1.5f}, {7,0.15f,1}, {0.2f,0.35f,0.75f}, VisualPartMotion::LeftAileron, {-13,1.5f,1}, {1,0,0}},
    {{13,1.5f,1.5f}, {7,0.15f,1}, {0.2f,0.35f,0.75f}, VisualPartMotion::RightAileron, {13,1.5f,1}, {1,0,0}},
    {{-6,1.5f,1.5f}, {6,0.15f,1}, {0.4f,0.5f,0.8f}, VisualPartMotion::Flap, {-6,1.5f,1}, {1,0,0}},
    {{6,1.5f,1.5f}, {6,0.15f,1}, {0.4f,0.5f,0.8f}, VisualPartMotion::Flap, {6,1.5f,1}, {1,0,0}},
    {{0,0.5f,9.5f}, {11,0.15f,1}, {0.2f,0.35f,0.75f}, VisualPartMotion::Elevator, {0,0.5f,9}, {1,0,0}},
    {{0,2,10}, {0.15f,3,1}, {0.2f,0.35f,0.75f}, VisualPartMotion::Rudder, {0,2,9.5f}, {0,1,0}},
    {{-4.1875f,-4.08f,1.433f}, {0.5f,1,0.8f}, {0.1f,0.1f,0.12f}},
    {{4.1875f,-4.08f,1.433f}, {0.5f,1,0.8f}, {0.1f,0.1f,0.12f}},
    {{0,-4.21f,-3.983f}, {0.5f,1,0.8f}, {0.1f,0.1f,0.12f}},
    {{0,0,-9.5f}, {0.15f,6,0.15f}, {0.15f,0.15f,0.18f}, VisualPartMotion::Rotor, {0,0,-9.5f}, {0,0,1}, 0},
}};

const std::array<AircraftDefinition, 2> Aircraft{{
    {AircraftType::F450, "f450", "F450 quadcopter", "F450", 0.55, 0, 0, 7, false, 0.002, Controls, F450Motors, F450DisabledSimulatorControls, F450Parts, Controllers},
    {AircraftType::C172, "c172", "Cessna 172", "c172x", 4.75, 0, 0, 65, true, 1.0 / 120, Controls, NoMotors, C172DisabledSimulatorControls, C172Parts, Controllers},
}};

} // namespace

std::span<const AircraftDefinition> GetAvailableAircraft() { return Aircraft; }

const AircraftDefinition &GetAircraftDefinition(AircraftType type) {
    for (const auto &aircraft : Aircraft) {
        if (aircraft.type == type) return aircraft;
    }
    throw std::invalid_argument("Unknown aircraft type");
}

std::optional<AircraftType> FindAircraftType(std::string_view name) {
    for (const auto &aircraft : Aircraft) {
        if (aircraft.command_name == name) return aircraft.type;
    }
    return std::nullopt;
}
