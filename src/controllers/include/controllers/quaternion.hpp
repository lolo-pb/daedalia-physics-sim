#pragma once

#include "sensors/sensor_types.hpp"

struct Quaternion {
    float w = 1.0f;
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
};

SensorVector3 NormalizeVector(const SensorVector3 &vector);

Quaternion NormalizeQuaternion(const Quaternion &quaternion);
Quaternion ConjugateQuaternion(const Quaternion &quaternion);
Quaternion MultiplyQuaternions(const Quaternion &left,
                               const Quaternion &right);
Quaternion QuaternionFromRotationVector(const SensorVector3 &rotation);
Quaternion QuaternionFromTwoUnitVectors(const SensorVector3 &from,
                                        const SensorVector3 &to);

SensorVector3 RotateVector(const Quaternion &quaternion,
                           const SensorVector3 &vector);
SensorVector3 QuaternionToRotationVector(const Quaternion &quaternion);
