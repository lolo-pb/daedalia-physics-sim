#pragma once

#include <array>
#include <glm/vec3.hpp>
#include <glm/gtc/quaternion.hpp>

// JSBSim native units and frames. No controller-facing unit conversion.
struct AircraftState {
    double timestamp_seconds = 0.0;
    glm::dvec3 position_ned_ft{};
    glm::dvec3 velocity_ned_ft_per_second{};
    glm::dvec3 body_rates_rad_per_second{};
    glm::dvec3 body_specific_force_ft_per_second_squared{};
    glm::dquat body_to_ned{1.0, 0.0, 0.0, 0.0};
    double pressure_psf = 0.0;
    double gravity_ft_per_second_squared = 0.0;
    double mass_slugs = 0.0;
    double altitude_asl_ft = 0.0;
    std::array<double, 4> propulsion_rpm{};
    double left_aileron_rad = 0.0;
    double right_aileron_rad = 0.0;
    double elevator_rad = 0.0;
    double rudder_rad = 0.0;
    double flap_rad = 0.0;
};
