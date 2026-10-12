#include "sensors/ideal_imu.hpp"
#include "physics/aircraft_state.hpp"

namespace {
SensorVector3 ToSensorVector(const glm::dvec3 &value) { return {value.x, value.y, value.z}; }
}

ImuSample IdealImuModel::Sample(const AircraftState &state) const {
    return {state.timestamp_seconds, ToSensorVector(state.body_rates_rad_per_second), ToSensorVector(state.body_specific_force_ft_per_second_squared)};
}
