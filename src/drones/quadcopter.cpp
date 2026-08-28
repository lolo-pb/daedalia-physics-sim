#include "drones/drone_definition.hpp"

DroneDefinition CreateQuadcopterDefinition() {
    return {
        JPH::Vec3(0.25f, 0.08f, 0.25f),
        1.0f,
        JPH::RVec3(0.0, 1.0, 0.0),
        JPH::Quat::sIdentity(),
        {
            {JPH::Vec3(-0.2f, 0.08f, -0.2f), JPH::Vec3(0.0f, 1.0f, 0.0f), JPH::Vec3(0.0f, 1.0f, 0.0f), 1000.0f, 5.0e-6f, 2.0e-8f},
            {JPH::Vec3( 0.2f, 0.08f, -0.2f), JPH::Vec3(0.0f, 1.0f, 0.0f), JPH::Vec3(0.0f,-1.0f, 0.0f), 1000.0f, 5.0e-6f, 2.0e-8f},
            {JPH::Vec3( 0.2f, 0.08f,  0.2f), JPH::Vec3(0.0f, 1.0f, 0.0f), JPH::Vec3(0.0f, 1.0f, 0.0f), 1000.0f, 5.0e-6f, 2.0e-8f},
            {JPH::Vec3(-0.2f, 0.08f,  0.2f), JPH::Vec3(0.0f, 1.0f, 0.0f), JPH::Vec3(0.0f,-1.0f, 0.0f), 1000.0f, 5.0e-6f, 2.0e-8f},
        },
    };
}
