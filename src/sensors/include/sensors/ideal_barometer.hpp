#pragma once

#include "sensors/sensor_types.hpp"
struct AircraftState;

class IdealBarometerModel {
public:
    BarometerSample Sample(const AircraftState &state) const;
};
