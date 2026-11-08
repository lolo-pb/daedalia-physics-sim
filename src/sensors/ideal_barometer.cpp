#include "sensors/ideal_barometer.hpp"
#include "physics/aircraft_state.hpp"

#include <cmath>

namespace {

constexpr double SeaLevelPressurePsf = 2116.22;
constexpr double AtmosphericScaleHeightFt = 28434.0;

} // namespace

double BarometricAltitudeFt(double pressure_psf) {
    return -AtmosphericScaleHeightFt *
           std::log(pressure_psf / SeaLevelPressurePsf);
}

BarometerSample IdealBarometerModel::Sample(const AircraftState &state) const {
    return {state.timestamp_seconds, state.pressure_psf};
}
