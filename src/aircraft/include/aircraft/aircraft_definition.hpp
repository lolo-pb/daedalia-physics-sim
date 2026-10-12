#pragma once

#include <array>
#include <optional>
#include <span>
#include <string_view>

enum class AircraftType { F450, C172 };
enum class ControllerType { Manual };
enum class VisualPartMotion { Fixed, Rotor, LeftAileron, RightAileron, Elevator, Rudder, Flap };

struct AircraftVisualPart {
    // Display frame: X right, Y up, -Z forward; all lengths in feet.
    std::array<float, 3> center_ft;
    std::array<float, 3> size_ft;
    std::array<float, 3> color;
    VisualPartMotion motion = VisualPartMotion::Fixed;
    std::array<float, 3> hinge_ft{};
    std::array<float, 3> axis{1.0f, 0.0f, 0.0f};
    int propulsion_index = 0;
    float base_yaw_rad = 0.0f;
};

struct AircraftDefinition {
    AircraftType type;
    std::string_view command_name;
    std::string_view display_name;
    std::string_view jsbsim_model;
    double initial_altitude_agl_ft;
    double latitude_deg;
    double longitude_deg;
    float camera_distance_ft;
    bool initialize_running;
    double max_integration_step_seconds;
    std::array<std::string_view, 7> control_properties;
    std::span<const std::string_view> motor_properties;
    std::span<const std::string_view> simulator_control_disable_properties;
    std::span<const AircraftVisualPart> visual_parts;
    std::span<const ControllerType> available_controllers;
};

inline constexpr AircraftType DefaultAircraftType = AircraftType::F450;
std::span<const AircraftDefinition> GetAvailableAircraft();
const AircraftDefinition &GetAircraftDefinition(AircraftType type);
std::optional<AircraftType> FindAircraftType(std::string_view name);
