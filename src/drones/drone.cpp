#include "drones/drone.hpp"

#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Body/BodyInterface.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>

namespace {

constexpr JPH::ObjectLayer MovingLayer = 1;

} // namespace

Drone::Drone(
    JPH::BodyInterface &bodies,
    const DroneDefinition &definition) :
    body_half_extent_(definition.body_half_extent),
    start_position_(definition.start_position),
    start_rotation_(definition.start_rotation) {
    motors_.reserve(definition.motors.size());
    for (const MotorDefinition &motor : definition.motors) {
        motors_.push_back({motor});
    }

    JPH::BodyCreationSettings settings(
        new JPH::BoxShape(body_half_extent_),
        start_position_,
        start_rotation_,
        JPH::EMotionType::Dynamic,
        MovingLayer);
    settings.mOverrideMassProperties = JPH::EOverrideMassProperties::CalculateInertia;
    settings.mMassPropertiesOverride.mMass = definition.mass;
    body_id_ = bodies.CreateAndAddBody(settings, JPH::EActivation::Activate);
}

JPH::BodyID Drone::GetBodyID() const {
    return body_id_;
}

std::size_t Drone::GetMotorCount() const {
    return motors_.size();
}

const JPH::Vec3 &Drone::GetBodyHalfExtent() const {
    return body_half_extent_;
}

void Drone::SetMotorTargets(const MotorCommands &motor_commands) {
    for (std::size_t index = 0; index < motors_.size(); ++index) {
        motors_[index].target = motor_commands.GetMotor(index);
    }
}

void Drone::UpdateMotors() {
    for (Motor &motor : motors_) {
        motor.speed_rad_per_second =
            motor.target * motor.definition.max_speed_rad_per_second;
    }
}

void Drone::Reset(JPH::BodyInterface &bodies) const {
    bodies.SetPositionAndRotation(
        body_id_,
        start_position_,
        start_rotation_,
        JPH::EActivation::Activate);
    bodies.SetLinearVelocity(body_id_, JPH::Vec3::sZero());
    bodies.SetAngularVelocity(body_id_, JPH::Vec3::sZero());
}

void Drone::ApplyForces(JPH::BodyInterface &bodies) {
    const JPH::RVec3 position = bodies.GetPosition(body_id_);
    const JPH::Quat rotation = bodies.GetRotation(body_id_);

    for (const Motor &motor : motors_) {
        const float speed_squared = motor.speed_rad_per_second * motor.speed_rad_per_second;
        const float thrust = motor.definition.thrust_coefficient * speed_squared;
        const float reaction_torque =
            motor.definition.reaction_torque_coefficient * speed_squared;
        const JPH::RVec3 motor_position = position
            + JPH::RVec3(rotation * motor.definition.local_position);
        bodies.AddForce(
            body_id_,
            rotation * motor.definition.local_thrust_direction * thrust,
            motor_position);
        bodies.AddTorque(
            body_id_,
            rotation * motor.definition.local_reaction_torque_direction
                * reaction_torque);
    }
}

std::vector<JPH::RVec3> Drone::GetMotorWorldPositions(
    const JPH::RVec3 &position,
    const JPH::Quat &rotation) const {
    std::vector<JPH::RVec3> world_positions;
    world_positions.reserve(motors_.size());
    for (const Motor &motor : motors_) {
        world_positions.push_back(
            position + JPH::RVec3(rotation * motor.definition.local_position));
    }
    return world_positions;
}
