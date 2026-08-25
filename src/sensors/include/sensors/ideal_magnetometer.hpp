#pragma once

#include <Jolt/Jolt.h>

#include "sensors/sensor_types.hpp"

class IdealMagnetometerModel {
public:
    MagnetometerSample Sample(
        double timestamp_seconds,
        const JPH::Quat &body_rotation) const;
};
