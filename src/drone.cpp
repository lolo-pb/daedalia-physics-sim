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
        {JPH::Vec3(-0.2f, 0.08f, -0.2f), JPH::Vec3(0.0f, 1.0f, 0.0f), JPH::Vec3(0.0f, 1.0f, 0.0f), 0.0f, 0.0f, 5.0f, 0.02f},
        {JPH::Vec3( 0.2f, 0.08f, -0.2f), JPH::Vec3(0.0f, 1.0f, 0.0f), JPH::Vec3(0.0f,-1.0f, 0.0f), 0.0f, 0.0f, 5.0f, 0.02f},
        {JPH::Vec3( 0.2f, 0.08f,  0.2f), JPH::Vec3(0.0f, 1.0f, 0.0f), JPH::Vec3(0.0f, 1.0f, 0.0f), 0.0f, 0.0f, 5.0f, 0.02f},
        {JPH::Vec3(-0.2f, 0.08f,  0.2f), JPH::Vec3(0.0f, 1.0f, 0.0f), JPH::Vec3(0.0f,-1.0f, 0.0f), 0.0f, 0.0f, 5.0f, 0.02f},
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

std::array<Motor, 4> &Drone::GetMotors() {
    return motors_;
}

const std::array<Motor, 4> &Drone::GetMotors() const {
    return motors_;
}

void Drone::Reset(JPH::BodyInterface &bodies) const {
    bodies.SetPositionAndRotation(body_id_, StartPosition, StartRotation, JPH::EActivation::Activate);
    bodies.SetLinearVelocity(body_id_, JPH::Vec3::sZero());
    bodies.SetAngularVelocity(body_id_, JPH::Vec3::sZero());
}

void Drone::ApplyForces(JPH::BodyInterface &bodies) const {
    const JPH::RVec3 position = bodies.GetPosition(body_id_);
    const JPH::Quat rotation = bodies.GetRotation(body_id_);

    for (const Motor &motor : motors_) {
        const float thrust = std::clamp(motor.thrust_newtons, 0.0f, motor.max_thrust_newtons);
        const float reaction_torque = std::clamp(
            motor.reaction_torque_newton_metres,
            0.0f,
            motor.max_reaction_torque_newton_metres);
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
