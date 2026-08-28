#include "drones/drone_definition.hpp"

#include <array>

namespace {

constexpr std::array<DroneOption, 1> AvailableDroneOptions{{
    {DroneType::Quadcopter, "quadcopter", "Quadcopter"},
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
    }
    return CreateQuadcopterDefinition();
}

DroneDefinition CreateQuadcopterDefinition() {
    return {
        JPH::Vec3(0.25f, 0.08f, 0.25f),
        1.0f,
        JPH::RVec3(0.0, 1.0, 0.0),
        JPH::Quat::sIdentity(),
        {
            {JPH::Vec3(-0.2f, 0.08f, -0.2f), JPH::Vec3(0.0f, 1.0f, 0.0f), JPH::Vec3(0.0f, 1.0f, 0.0f), 1000.0f, 5.0e-6f, 2.0e-8f},
            {JPH::Vec3( 0.2f, 0.08f, -0.2f), JPH::Vec3(0.0f, 1.0f, 0.0f), JPH::Vec3(0.0f,-1.0f, 0.0f), 1000.0f, 5.0e-6f, 2.0e-8f},
            {JPH::Vec3( 0.2f, 0.08f,  0.2f), JPH::Vec3(0.0f, 1.0f, 0.0f), JPH::Vec3(0.0f, 1.0f, 0.0f), 1000.0f, 5.0e-6f, 2.0e-8f},
            {JPH::Vec3(-0.2f, 0.08f,  0.2f), JPH::Vec3(0.0f, 1.0f, 0.0f), JPH::Vec3(0.0f,-1.0f, 0.0f), 1000.0f, 5.0e-6f, 2.0e-8f},
        },
    };
}
