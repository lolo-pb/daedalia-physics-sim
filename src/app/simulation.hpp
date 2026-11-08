#pragma once

#include <array>
#include <memory>
#include "aircraft/aircraft_definition.hpp"
#include "controllers/controller_io.hpp"
#include "physics/aircraft_state.hpp"

struct AircraftRenderState {
    glm::dvec3 position_ft{};
    glm::dquat rotation{1.0, 0.0, 0.0, 0.0};
    double timestamp_seconds = 0.0;
    std::array<double, 4> rotor_angles_rad{};
    double left_aileron_rad = 0, right_aileron_rad = 0;
    double elevator_rad = 0, rudder_rad = 0, flap_rad = 0;
};

class Simulation {
public:
    explicit Simulation(const AircraftDefinition &definition);
    ~Simulation();
    Simulation(const Simulation &) = delete;
    Simulation &operator=(const Simulation &) = delete;
    void SelectController(int slot);
    void Step(const ControllerKeys &keys);
    void Reset();
    int RunSmokeTest();
    ControllerType GetActiveController() const;
    float GetActiveControllerThrottle() const;
    const ControllerOutput &GetLatestCommands() const;
    const AircraftDefinition &GetAircraftDefinition() const;
    int GetPhysicsFrequencyHz() const;
    double GetPhysicsStepSeconds() const;
    void SetPhysicsFrequencyHz(int frequency_hz);
    const AircraftState &InspectAircraft() const;
    glm::dvec3 GetAircraftPosition() const;
    AircraftRenderState GetAircraftRenderState() const;
    const ImuSample &GetLatestImuSample() const;
    const GpsSample &GetLatestGpsSample() const;
    const BarometerSample &GetLatestBarometerSample() const;
    const MagnetometerSample &GetLatestMagnetometerSample() const;
private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
