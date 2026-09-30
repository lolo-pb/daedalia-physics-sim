#pragma once

#include "sensors/sensor_types.hpp"

struct Quaternion {
    float w = 1.0f;
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
};

// Returns a unit-length vector, or zero when the input is too small.
SensorVector3 NormalizeVector(const SensorVector3 &vector);

// Returns a unit-length quaternion, or identity when the input is too small.
Quaternion NormalizeQuaternion(const Quaternion &quaternion);
// Returns the conjugate, which reverses a unit quaternion's rotation.
Quaternion ConjugateQuaternion(const Quaternion &quaternion);
// Combines two quaternion rotations into one rotation.
Quaternion MultiplyQuaternions(const Quaternion &left,
                               const Quaternion &right);
// Converts an axis-angle rotation vector into a quaternion.
Quaternion QuaternionFromRotationVector(const SensorVector3 &rotation);
// Returns the shortest rotation from one unit vector to another.
Quaternion QuaternionFromTwoUnitVectors(const SensorVector3 &from,
                                        const SensorVector3 &to);

// Rotates a vector by a quaternion.
SensorVector3 RotateVector(const Quaternion &quaternion,
                           const SensorVector3 &vector);
// Converts a quaternion into its shortest axis-angle rotation vector.
SensorVector3 QuaternionToRotationVector(const Quaternion &quaternion);
