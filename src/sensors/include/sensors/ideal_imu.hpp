#pragma once

#include "sensors/sensor_types.hpp"
struct AircraftState;

class IdealImuModel {
public:
    ImuSample Sample(const AircraftState &state) const;
};
