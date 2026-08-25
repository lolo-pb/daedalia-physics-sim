#pragma once

#include <Jolt/Jolt.h>

#include "sensors/sensor_types.hpp"

class IdealImuModel {
public:
    void Reset(const JPH::Vec3 &world_linear_velocity);

    ImuSample Sample(
        double timestamp_seconds,
        float timestep_seconds,
        const JPH::Quat &body_rotation,
        const JPH::Vec3 &world_angular_velocity,
        const JPH::Vec3 &world_linear_velocity,
        const JPH::Vec3 &world_gravity);

private:
    JPH::Vec3 previous_world_linear_velocity_ = JPH::Vec3::sZero();
    double previous_timestamp_seconds_ = 0.0;
    bool has_previous_velocity_ = false;
};
