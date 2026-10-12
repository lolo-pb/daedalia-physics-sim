#include "sensors/ideal_barometer.hpp"
#include "physics/aircraft_state.hpp"

BarometerSample IdealBarometerModel::Sample(const AircraftState &state) const {
    return {state.timestamp_seconds, state.pressure_psf};
}
