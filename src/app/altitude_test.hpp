#pragma once

#include "aircraft/aircraft_definition.hpp"

void RunAltitudeTest(const AircraftDefinition &aircraft, std::string_view controller, double duration_seconds, bool plot);
