#include "controllers/quaternion.hpp"

#include <algorithm>
#include <cmath>

namespace {

constexpr float MinimumLengthSquared = 1.0e-12f;

float Dot(const SensorVector3 &left, const SensorVector3 &right) {
    return left.x * right.x + left.y * right.y + left.z * right.z;
}

SensorVector3 Cross(const SensorVector3 &left, const SensorVector3 &right) {
    return {
        left.y * right.z - left.z * right.y,
        left.z * right.x - left.x * right.z,
        left.x * right.y - left.y * right.x,
    };
}

float LengthSquared(const SensorVector3 &vector) { return Dot(vector, vector); }

} // namespace

SensorVector3 NormalizeVector(const SensorVector3 &vector) {
    const float length_squared = LengthSquared(vector);
    if (length_squared <= MinimumLengthSquared) {
        return {};
    }

    const float inverse_length = 1.0f / std::sqrt(length_squared);
    return {
        vector.x * inverse_length,
        vector.y * inverse_length,
        vector.z * inverse_length,
    };
}

Quaternion NormalizeQuaternion(const Quaternion &quaternion) {
    const float length_squared = quaternion.w * quaternion.w +
                                 quaternion.x * quaternion.x +
                                 quaternion.y * quaternion.y +
                                 quaternion.z * quaternion.z;
    if (length_squared <= MinimumLengthSquared) {
        return {};
    }

    const float inverse_length = 1.0f / std::sqrt(length_squared);
    return {
        quaternion.w * inverse_length,
        quaternion.x * inverse_length,
        quaternion.y * inverse_length,
        quaternion.z * inverse_length,
    };
}

Quaternion ConjugateQuaternion(const Quaternion &quaternion) {
    return {
        quaternion.w,
        -quaternion.x,
        -quaternion.y,
        -quaternion.z,
    };
}

Quaternion MultiplyQuaternions(const Quaternion &left,
                               const Quaternion &right) {
    return {
        left.w * right.w - left.x * right.x - left.y * right.y -
            left.z * right.z,
        left.w * right.x + left.x * right.w + left.y * right.z -
            left.z * right.y,
        left.w * right.y - left.x * right.z + left.y * right.w +
            left.z * right.x,
        left.w * right.z + left.x * right.y - left.y * right.x +
            left.z * right.w,
    };
}

Quaternion QuaternionFromRotationVector(const SensorVector3 &rotation) {
    const float angle_squared = LengthSquared(rotation);
    if (angle_squared <= MinimumLengthSquared) {
        return NormalizeQuaternion({
            1.0f,
            0.5f * rotation.x,
            0.5f * rotation.y,
            0.5f * rotation.z,
        });
    }

    const float angle = std::sqrt(angle_squared);
    const float half_angle = 0.5f * angle;
    const float vector_scale = std::sin(half_angle) / angle;
    return {
        std::cos(half_angle),
        rotation.x * vector_scale,
        rotation.y * vector_scale,
        rotation.z * vector_scale,
    };
}

Quaternion QuaternionFromTwoUnitVectors(const SensorVector3 &from,
                                        const SensorVector3 &to) {
    const float dot = std::clamp(Dot(from, to), -1.0f, 1.0f);
    if (dot < -0.999999f) {
        const SensorVector3 reference =
            std::abs(from.x) < std::abs(from.y) ? SensorVector3{1.0f, 0.0f, 0.0f}
                                                : SensorVector3{0.0f, 1.0f, 0.0f};
        const SensorVector3 axis = NormalizeVector(Cross(from, reference));
        return {0.0f, axis.x, axis.y, axis.z};
    }

    const SensorVector3 cross = Cross(from, to);
    return NormalizeQuaternion({1.0f + dot, cross.x, cross.y, cross.z});
}

SensorVector3 RotateVector(const Quaternion &quaternion,
                           const SensorVector3 &vector) {
    const Quaternion rotated = MultiplyQuaternions(
        MultiplyQuaternions(quaternion, {0.0f, vector.x, vector.y, vector.z}),
        ConjugateQuaternion(quaternion));
    return {rotated.x, rotated.y, rotated.z};
}

SensorVector3 QuaternionToRotationVector(const Quaternion &quaternion) {
    Quaternion shortest_rotation = NormalizeQuaternion(quaternion);
    if (shortest_rotation.w < 0.0f) {
        shortest_rotation.w = -shortest_rotation.w;
        shortest_rotation.x = -shortest_rotation.x;
        shortest_rotation.y = -shortest_rotation.y;
        shortest_rotation.z = -shortest_rotation.z;
    }

    const float vector_length = std::sqrt(
        shortest_rotation.x * shortest_rotation.x +
        shortest_rotation.y * shortest_rotation.y +
        shortest_rotation.z * shortest_rotation.z);
    if (vector_length <= MinimumLengthSquared) {
        return {
            2.0f * shortest_rotation.x,
            2.0f * shortest_rotation.y,
            2.0f * shortest_rotation.z,
        };
    }

    const float angle = 2.0f * std::atan2(vector_length, shortest_rotation.w);
    const float scale = angle / vector_length;
    return {
        shortest_rotation.x * scale,
        shortest_rotation.y * scale,
        shortest_rotation.z * scale,
    };
}
