#include "sensors/ideal_magnetometer.hpp"

MagnetometerSample IdealMagnetometerModel::Sample(
    double timestamp_seconds,
    const JPH::Quat &body_rotation) const {
    const JPH::Vec3 world_magnetic_field_microteslas(0.0f, 0.0f, -50.0f);
    const JPH::Vec3 body_field = body_rotation.Conjugated() * world_magnetic_field_microteslas;
    return {
        timestamp_seconds,
        {body_field.GetX(), body_field.GetY(), body_field.GetZ()},
    };
}
