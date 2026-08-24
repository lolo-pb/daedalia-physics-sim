#pragma once

#include <array>

#include <Jolt/Jolt.h>
#include <Jolt/Physics/Body/BodyID.h>

namespace JPH {
class BodyInterface;
}

struct Motor {
    JPH::Vec3 local_position;
    JPH::Vec3 local_thrust_direction;
    JPH::Vec3 local_reaction_torque_direction;

    float thrust_newtons = 0.0f;
    float reaction_torque_newton_metres = 0.0f;

    float max_thrust_newtons = 0.0f;
    float max_reaction_torque_newton_metres = 0.0f;
};

class Drone {
public:
    explicit Drone(JPH::BodyInterface &bodies);

    JPH::BodyID GetBodyID() const;
    std::array<Motor, 4> &GetMotors();
    const std::array<Motor, 4> &GetMotors() const;

    void Reset(JPH::BodyInterface &bodies) const;
    void ApplyForces(JPH::BodyInterface &bodies) const;
    std::array<JPH::RVec3, 4> GetMotorWorldPositions(const JPH::RVec3 &position, const JPH::Quat &rotation) const;

private:
    JPH::BodyID body_id_;
    std::array<Motor, 4> motors_;
};
