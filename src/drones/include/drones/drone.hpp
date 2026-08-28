#pragma once

#include <cstddef>
#include <vector>

#include <Jolt/Jolt.h>
#include <Jolt/Physics/Body/BodyID.h>

#include "controllers/controller_io.hpp"
#include "drones/drone_definition.hpp"

namespace JPH {
class BodyInterface;
}

class Drone {
public:
    Drone(JPH::BodyInterface &bodies, const DroneDefinition &definition);

    JPH::BodyID GetBodyID() const;
    std::size_t GetMotorCount() const;
    const JPH::Vec3 &GetBodyHalfExtent() const;
    void SetMotorTargets(const MotorCommands &motor_commands);
    void UpdateMotors();

    void Reset(JPH::BodyInterface &bodies) const;
    void ApplyForces(JPH::BodyInterface &bodies);
    std::vector<JPH::RVec3> GetMotorWorldPositions(
        const JPH::RVec3 &position,
        const JPH::Quat &rotation) const;

private:
    struct Motor {
        MotorDefinition definition;
        float target = 0.0f;
        float speed_rad_per_second = 0.0f;
    };

    JPH::BodyID body_id_;
    JPH::Vec3 body_half_extent_;
    JPH::RVec3 start_position_;
    JPH::Quat start_rotation_;
    std::vector<Motor> motors_;
};
