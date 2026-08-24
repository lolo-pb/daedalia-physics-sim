#include "sensors/ideal_imu.hpp"

namespace {

ImuVector3 ToImuVector(const JPH::Vec3 &vector) {
    return {vector.GetX(), vector.GetY(), vector.GetZ()};
}

} // namespace

void IdealImuModel::Reset(const JPH::Vec3 &world_linear_velocity) {
    previous_world_linear_velocity_ = world_linear_velocity;
    has_previous_velocity_ = false;
}

ImuSample IdealImuModel::Sample(
    double timestamp_seconds,
    float timestep_seconds,
    const JPH::Quat &body_rotation,
    const JPH::Vec3 &world_angular_velocity,
    const JPH::Vec3 &world_linear_velocity,
    const JPH::Vec3 &world_gravity) {
    const double elapsed_seconds = timestamp_seconds - previous_timestamp_seconds_;
    const float acceleration_timestep = has_previous_velocity_ && elapsed_seconds > 0.0
        ? static_cast<float>(elapsed_seconds)
        : timestep_seconds;
    const JPH::Vec3 world_acceleration =
        (world_linear_velocity - previous_world_linear_velocity_) / acceleration_timestep;
    previous_world_linear_velocity_ = world_linear_velocity;
    previous_timestamp_seconds_ = timestamp_seconds;
    has_previous_velocity_ = true;

    const JPH::Quat world_to_body = body_rotation.Conjugated();
    return {
        timestamp_seconds,
        ToImuVector(world_to_body * world_angular_velocity),
        ToImuVector(world_to_body * (world_acceleration - world_gravity)),
    };
}
