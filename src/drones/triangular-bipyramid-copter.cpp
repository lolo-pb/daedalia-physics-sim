#include "drones/drone_definition.hpp"


DroneDefinition CreateTriangularBipyramidCopterDefinition() {
  return {
      JPH::Vec3(0.25f, 0.08f, 0.25f), // half extent 
      0.5f,// wheight
      JPH::RVec3(0.0, 1.0, 0.0),//start pos
      JPH::Quat::sIdentity(),   //start rPot
      {// list of motors
        //{JPH::Vec3(-0.2165f, 0.08f, -0.125f),  // local position
        // JPH::Vec3(0.0f, 1.0f, 0.0f),          // local thrust direction
        // JPH::Vec3(0.0f, 1.0f, 0.0f),          // local reaction torque direction
        // 1000.0f,    // max speed 
        // 5.0e-6f,    // thrust coefficient
        // 2.0e-8f     // reaction torque coefficient
        //},        
        {JPH::Vec3( 0.096,  0.167,  0.272),
         JPH::Vec3(0.0f, 0.0f, 0.0f),
         JPH::Vec3(0.0f, 0.0f, 0.0f),
         1000.0f,
         5.0e-6f,
         2.0e-8f
        },
        {JPH::Vec3(-0.192,  0.000,  0.272),
         JPH::Vec3(0.0f, 0.0f, 0.0f),
         JPH::Vec3(0.0f, 0.0f, 0.0f),
         1000.0f,
         5.0e-6f,
         2.0e-8f
        },
        {JPH::Vec3( 0.096, -0.167,  0.272),
         JPH::Vec3(0.0f, 0.0f, 0.0f),
         JPH::Vec3(0.0f, 0.0f, 0.0f),
         1000.0f,
         5.0e-6f,
         2.0e-8f
        },
        {JPH::Vec3( 0.096,  0.167, -0.272),
         JPH::Vec3(0.0f, 0.0f, 0.0f),
         JPH::Vec3(0.0f, 0.0f, 0.0f),
         1000.0f,
         5.0e-6f,
         2.0e-8f
        },
        {JPH::Vec3(-0.192,  0.000, -0.272),
         JPH::Vec3(0.0f, 0.0f, 0.0f),
         JPH::Vec3(0.0f, 0.0f, 0.0f),
         1000.0f,
         5.0e-6f,
         2.0e-8f
        },
        {JPH::Vec3( 0.096, -0.167, -0.272),
         JPH::Vec3(0.0f, 0.0f, 0.0f),
         JPH::Vec3(0.0f, 0.0f, 0.0f),
         1000.0f,
         5.0e-6f,
         2.0e-8f
        }
      },
  };
}


/*
//esto es geomettria
Base vertex 1: ( 0.577,  0.000,  0.000)
Base vertex 2: (-0.289,  0.500,  0.000)
Base vertex 3: (-0.289, -0.500,  0.000)
Top apex:      ( 0.000,  0.000,  0.816)
Bottom apex:   ( 0.000,  0.000, -0.816)

//aca van los motores
Upper face 1: ( 0.096,  0.167,  0.272)X
Upper face 2: (-0.192,  0.000,  0.272)X
Upper face 3: ( 0.096, -0.167,  0.272)X
Lower face 1: ( 0.096,  0.167, -0.272)X
Lower face 2: (-0.192,  0.000, -0.272)X
Lower face 3: ( 0.096, -0.167, -0.272)X
*/