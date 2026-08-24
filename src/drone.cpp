#include "drone.hpp"

#include <algorithm>

#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Body/BodyInterface.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>

namespace {

constexpr JPH::ObjectLayer MovingLayer = 1;
const JPH::RVec3 StartPosition(0.0, 1.0, 0.0);
const JPH::Quat StartRotation = JPH::Quat::sIdentity();

} // namespace

Drone::Drone(JPH::BodyInterface &bodies) :
    motors_{{
        {JPH::Vec3(-0.2f, 0.08f, -0.2f), JPH::Vec3(0.0f, 1.0f, 0.0f), JPH::Vec3(0.0f, 1.0f, 0.0f), 0.0f, 0.0f, 1000.0f, 5.0e-6f, 2.0e-8f},
        {JPH::Vec3( 0.2f, 0.08f, -0.2f), JPH::Vec3(0.0f, 1.0f, 0.0f), JPH::Vec3(0.0f,-1.0f, 0.0f), 0.0f, 0.0f, 1000.0f, 5.0e-6f, 2.0e-8f},
        {JPH::Vec3( 0.2f, 0.08f,  0.2f), JPH::Vec3(0.0f, 1.0f, 0.0f), JPH::Vec3(0.0f, 1.0f, 0.0f), 0.0f, 0.0f, 1000.0f, 5.0e-6f, 2.0e-8f},
        {JPH::Vec3(-0.2f, 0.08f,  0.2f), JPH::Vec3(0.0f, 1.0f, 0.0f), JPH::Vec3(0.0f,-1.0f, 0.0f), 0.0f, 0.0f, 1000.0f, 5.0e-6f, 2.0e-8f},
    }} {
    JPH::BodyCreationSettings settings(
        new JPH::BoxShape(JPH::Vec3(0.25f, 0.08f, 0.25f)),
        StartPosition,
        StartRotation,
        JPH::EMotionType::Dynamic,
        MovingLayer);
    settings.mOverrideMassProperties = JPH::EOverrideMassProperties::CalculateInertia;
    settings.mMassPropertiesOverride.mMass = 1.0f;
    body_id_ = bodies.CreateAndAddBody(settings, JPH::EActivation::Activate);
}

JPH::BodyID Drone::GetBodyID() const {
    return body_id_;
}

void Drone::SetMotorCommand(MotorId id, float command) {
    motors_[static_cast<std::size_t>(id)].command = std::clamp(command, 0.0f, 1.0f);
}

void Drone::Reset(JPH::BodyInterface &bodies) const {
    bodies.SetPositionAndRotation(body_id_, StartPosition, StartRotation, JPH::EActivation::Activate);
    bodies.SetLinearVelocity(body_id_, JPH::Vec3::sZero());
    bodies.SetAngularVelocity(body_id_, JPH::Vec3::sZero());
}

void Drone::ApplyForces(JPH::BodyInterface &bodies) {
    const JPH::RVec3 position = bodies.GetPosition(body_id_);
    const JPH::Quat rotation = bodies.GetRotation(body_id_);

    for (Motor &motor : motors_) {
        motor.speed_rad_per_second = std::clamp(motor.command, 0.0f, 1.0f) * motor.max_speed_rad_per_second;
        const float speed_squared = motor.speed_rad_per_second * motor.speed_rad_per_second;
        const float thrust = motor.thrust_coefficient * speed_squared;
        const float reaction_torque = motor.reaction_torque_coefficient * speed_squared;
        const JPH::RVec3 motor_position = position + JPH::RVec3(rotation * motor.local_position);
        bodies.AddForce(body_id_, rotation * motor.local_thrust_direction * thrust, motor_position);
        bodies.AddTorque(body_id_, rotation * motor.local_reaction_torque_direction * reaction_torque);
    }
}

std::array<JPH::RVec3, 4> Drone::GetMotorWorldPositions(const JPH::RVec3 &position, const JPH::Quat &rotation) const {
    std::array<JPH::RVec3, 4> world_positions;
    for (size_t index = 0; index < motors_.size(); ++index) {
        world_positions[index] = position + JPH::RVec3(rotation * motors_[index].local_position);
    }
    return world_positions;
}
