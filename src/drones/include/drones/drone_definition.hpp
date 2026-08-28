#pragma once

#include <optional>
#include <span>
#include <string_view>
#include <vector>

#include <Jolt/Jolt.h>

struct MotorDefinition {
    JPH::Vec3 local_position;
    JPH::Vec3 local_thrust_direction;
    JPH::Vec3 local_reaction_torque_direction;
    float max_speed_rad_per_second = 0.0f;
    float thrust_coefficient = 0.0f;
    float reaction_torque_coefficient = 0.0f;
};

struct DroneDefinition {
    JPH::Vec3 body_half_extent;
    float mass = 0.0f;
    JPH::RVec3 start_position;
    JPH::Quat start_rotation;
    std::vector<MotorDefinition> motors;
};

enum class DroneType {
    Quadcopter,
};

struct DroneOption {
    DroneType type;
    std::string_view command_name;
    std::string_view display_name;
};

inline constexpr DroneType DefaultDroneType = DroneType::Quadcopter;

std::span<const DroneOption> GetAvailableDroneOptions();
std::optional<DroneType> FindAvailableDroneType(std::string_view command_name);
DroneDefinition CreateDroneDefinition(DroneType type);
DroneDefinition CreateQuadcopterDefinition();
