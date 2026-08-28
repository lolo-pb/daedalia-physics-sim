#pragma once

#include <array>
#include <memory>
#include <vector>

#include <Jolt/Jolt.h>

#include "controllers/controller_io.hpp"
#include "controllers/controller_selection.hpp"
#include "sensors/sensor_types.hpp"

struct DroneDefinition;

class PhysicsRuntime {
public:
    PhysicsRuntime();
    ~PhysicsRuntime();

    PhysicsRuntime(const PhysicsRuntime &) = delete;
    PhysicsRuntime &operator=(const PhysicsRuntime &) = delete;
};

struct DroneInspection {
    JPH::RVec3 position;
    JPH::Quat rotation;
    JPH::Vec3 linear_velocity;
    JPH::Vec3 angular_velocity;
    float mass = 0.0f;
};

struct DroneRenderState {
    JPH::RVec3 position;
    JPH::Quat rotation;
    JPH::Vec3 body_half_extent;
    std::vector<JPH::RVec3> motor_positions;
};

class Simulation {
public:
    explicit Simulation(const DroneDefinition &drone_definition);
    ~Simulation();

    Simulation(const Simulation &) = delete;
    Simulation &operator=(const Simulation &) = delete;

    void SelectController(int slot);
    void Step(const ControllerKeys &controller_keys);
    void Reset();

    int RunSmokeTest();
    int RunQuadcopterControlSmokeTest();

    FlightController GetActiveController() const;
    float GetActiveControllerThrottle() const;

    int GetPhysicsFrequencyHz() const;
    double GetPhysicsStepSeconds() const;
    void SetPhysicsFrequencyHz(int frequency_hz);

    std::array<float, 3> GetGravity() const;
    void SetGravity(const std::array<float, 3> &gravity);

    JPH::RVec3 GetDronePosition() const;
    DroneInspection InspectDrone() const;
    DroneRenderState GetDroneRenderState() const;

    const ImuSample &GetLatestImuSample() const;
    const GpsSample &GetLatestGpsSample() const;
    const BarometerSample &GetLatestBarometerSample() const;
    const MagnetometerSample &GetLatestMagnetometerSample() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
