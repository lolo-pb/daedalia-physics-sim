#include "sensors/ideal_gps.hpp"
#include "physics/aircraft_state.hpp"

namespace {
SensorVector3 ToSensorVector(const glm::dvec3 &value) { return {value.x, value.y, value.z}; }
}

GpsSample IdealGpsModel::Sample(const AircraftState &state) const {
    return {state.timestamp_seconds, ToSensorVector(state.position_ned_ft), ToSensorVector(state.velocity_ned_ft_per_second)};
}
