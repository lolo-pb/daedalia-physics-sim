#pragma once

#include "sensors/sensor_types.hpp"
struct AircraftState;

class IdealMagnetometerModel {
public:
    MagnetometerSample Sample(const AircraftState &state) const;
};
