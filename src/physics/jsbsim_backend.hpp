#pragma once

#include <memory>
#include "aircraft/aircraft_definition.hpp"
#include "controllers/controller_io.hpp"
#include "physics/aircraft_state.hpp"

class JsbsimBackend {
public:
    explicit JsbsimBackend(const AircraftDefinition &definition);
    ~JsbsimBackend();
    void Reset(double timestep_seconds);
    void Step(const ControllerOutput &commands);
    void SetTimestep(double seconds);
    AircraftState ReadState() const;
private:
    struct Impl;
    const AircraftDefinition &definition_;
    std::unique_ptr<Impl> impl_;
};
