#include "drones/drone_definition.hpp"


DroneDefinition CreateTriangularBipyramidCopterDefinition() {
  return {
      JPH::Vec3(0.25f, 0.08f, 0.25f), // half extent 
      0.5f,// wheight
      JPH::RVec3(0.0, 1.0, 0.0),//start pos
      JPH::Quat::sIdentity(),   //start rPot
      {// list of motors
        {JPH::Vec3(-0.2165f, 0.08f, -0.125f),  // local position
         JPH::Vec3(0.0f, 1.0f, 0.0f),          // local thrust direction
         JPH::Vec3(0.0f, 1.0f, 0.0f),          // local reaction torque direction
         1000.0f,    // max speed 
         5.0e-6f,    // thrust coefficient
         2.0e-8f     // reaction torque coefficient
        },
        {JPH::Vec3(0.2165f, 0.08f, -0.125f), JPH::Vec3(0.0f, 1.0f, 0.0f),
         JPH::Vec3(0.0f, 1.0f, 0.0f), 
         1000.0f, 5.0e-6f, 2.0e-8f
        },
        {JPH::Vec3(0.0f, 0.08f, 0.25f), JPH::Vec3(0.0f, 1.0f, 0.0f),
         JPH::Vec3(0.0f, -1.0f, 0.0f), 
         1000.0f, 5.0e-6f, 2.0e-8f
        },
      },
  };
}
