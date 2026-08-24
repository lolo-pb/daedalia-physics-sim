#pragma once

#include <array>
#include <cstddef>

#include <Jolt/Jolt.h>
#include <Jolt/Physics/Body/BodyID.h>

namespace JPH {
class BodyInterface;
}

enum class MotorId {
    FrontLeft,
    FrontRight,
    RearRight,
    RearLeft,
};

class Drone {
public:
    explicit Drone(JPH::BodyInterface &bodies);

    JPH::BodyID GetBodyID() const;
    void SetMotorCommand(MotorId id, float command);

    void Reset(JPH::BodyInterface &bodies) const;
    void ApplyForces(JPH::BodyInterface &bodies);
    std::array<JPH::RVec3, 4> GetMotorWorldPositions(const JPH::RVec3 &position, const JPH::Quat &rotation) const;

private:
    struct Motor {
        JPH::Vec3 local_position;
        JPH::Vec3 local_thrust_direction;
        JPH::Vec3 local_reaction_torque_direction;

        float command = 0.0f;
        float speed_rad_per_second = 0.0f;
        float max_speed_rad_per_second = 0.0f;
        float thrust_coefficient = 0.0f;
        float reaction_torque_coefficient = 0.0f;
    };

    JPH::BodyID body_id_;
    std::array<Motor, 4> motors_;
};
