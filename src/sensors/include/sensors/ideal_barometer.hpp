#pragma once

#include "sensors/sensor_types.hpp"

class IdealBarometerModel {
public:
    BarometerSample Sample(double timestamp_seconds, float world_altitude_meters) const;
};

float BarometricAltitudeMeters(float pressure_pascals);
