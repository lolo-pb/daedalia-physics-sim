#pragma once

#include "sensors/sensor_types.hpp"
struct AircraftState;

class IdealGpsModel {
public:
    GpsSample Sample(const AircraftState &state) const;
};
