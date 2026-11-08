#include "sensors/ideal_magnetometer.hpp"
#include "physics/aircraft_state.hpp"

namespace {
SensorVector3 ToSensorVector(const glm::dvec3 &value) { return {value.x, value.y, value.z}; }
}

MagnetometerSample IdealMagnetometerModel::Sample(const AircraftState &state) const {
    // Ideal 50 uT north-pointing field in NED, rotated into body axes.
    const glm::dvec3 field = glm::conjugate(state.body_to_ned) * glm::dvec3(50, 0, 0);
    return {state.timestamp_seconds, ToSensorVector(field)};
}
