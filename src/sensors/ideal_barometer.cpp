#include "sensors/ideal_barometer.hpp"

#include <cmath>

namespace {

constexpr float SeaLevelPressurePascals = 101325.0f;
constexpr float AtmosphericScaleHeightMeters = 8434.5f;

} // namespace

BarometerSample IdealBarometerModel::Sample(
    double timestamp_seconds,
    float world_altitude_meters) const {
    return {
        timestamp_seconds,
        SeaLevelPressurePascals
            * std::exp(-world_altitude_meters / AtmosphericScaleHeightMeters),
    };
}

float BarometricAltitudeMeters(float pressure_pascals) {
    return -AtmosphericScaleHeightMeters
        * std::log(pressure_pascals / SeaLevelPressurePascals);
}
