#pragma once

#include "sensors/sensor_types.hpp"
struct AircraftState;

double BarometricAltitudeFt(double pressure_psf);

class IdealBarometerModel {
public:
    BarometerSample Sample(const AircraftState &state) const;
};
