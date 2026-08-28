#include "drones/drone_definition.hpp"

#include <array>

namespace {

constexpr std::array<DroneOption, 2> AvailableDroneOptions{{
    {DroneType::Quadcopter, "quadcopter", "Quadcopter"},
    {DroneType::Tricopter, "tricopter", "Tricopter"},
}};

} // namespace

std::span<const DroneOption> GetAvailableDroneOptions() {
    return AvailableDroneOptions;
}

std::optional<DroneType> FindAvailableDroneType(
    std::string_view command_name) {
    for (const DroneOption &option : AvailableDroneOptions) {
        if (option.command_name == command_name) {
            return option.type;
        }
    }
    return std::nullopt;
}

DroneDefinition CreateDroneDefinition(DroneType type) {
    switch (type) {
    case DroneType::Quadcopter:
        return CreateQuadcopterDefinition();
    case DroneType::Tricopter:
        return CreateTricopterDefinition();
    }
    return CreateQuadcopterDefinition();
}
