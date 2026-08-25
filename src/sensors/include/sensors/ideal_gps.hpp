#pragma once

#include <Jolt/Jolt.h>

#include "sensors/sensor_types.hpp"

class IdealGpsModel {
public:
    GpsSample Sample(
        double timestamp_seconds,
        const JPH::RVec3 &world_position,
        const JPH::Vec3 &world_velocity) const;
};
