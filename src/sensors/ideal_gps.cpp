#include "sensors/ideal_gps.hpp"

GpsSample IdealGpsModel::Sample(
    double timestamp_seconds,
    const JPH::RVec3 &world_position,
    const JPH::Vec3 &world_velocity) const {
    return {
        timestamp_seconds,
        {
            static_cast<float>(world_position.GetX()),
            static_cast<float>(world_position.GetY()),
            static_cast<float>(world_position.GetZ()),
        },
        {
            world_velocity.GetX(),
            world_velocity.GetY(),
            world_velocity.GetZ(),
        },
    };
}
