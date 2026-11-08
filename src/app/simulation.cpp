#include "simulation.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <numbers>
#include <stdexcept>

#include "controllers/c172_direct_controller.hpp"
#include "controllers/demo_controller.hpp"
#include "controllers/angle_mode_controller.hpp"
#include "controllers/horizon_mode_controller.hpp"
#include "controllers/position_hold_controller.hpp"
#include "physics/jsbsim_backend.hpp"
#include "sensors/ideal_imu.hpp"
#include "sensors/ideal_gps.hpp"
#include "sensors/ideal_barometer.hpp"
#include "sensors/ideal_magnetometer.hpp"

namespace {
// NED (north,east,down) and body (forward,right,down) -> display (right,up,back).
glm::dmat3 DisplayBasis() {
    return glm::dmat3(glm::dvec3(0,0,-1), glm::dvec3(1,0,0), glm::dvec3(0,-1,0));
}
bool Finite(const glm::dvec3 &v) { return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z); }
}

struct Simulation::Impl {
    const AircraftDefinition &definition;
    JsbsimBackend backend;
    DemoController f450_demo;
    AngleModeController f450_angle;
    HorizonModeController f450_horizon;
    PositionHoldController f450_position_hold;
    C172DirectController c172_direct;
    ControllerType active = ControllerType::F450Demo;
    ControllerOutput commands;
    AircraftState state;
    IdealImuModel imu;
    IdealGpsModel gps;
    IdealBarometerModel barometer;
    IdealMagnetometerModel magnetometer;
    ImuSample imu_sample;
    GpsSample gps_sample;
    BarometerSample barometer_sample;
    MagnetometerSample magnetometer_sample;
    std::array<double, 4> rotor_angles{};
    int frequency_hz = 60;

    explicit Impl(const AircraftDefinition &aircraft)
        : definition(aircraft), backend(aircraft), commands(aircraft.motor_properties.size()) {
        if (definition.available_controllers.empty()) throw std::runtime_error("Aircraft has no controllers");
        active = definition.available_controllers.front();
        Refresh();
    }
    void Refresh() {
        state = backend.ReadState();
        imu_sample = imu.Sample(state);
        gps_sample = gps.Sample(state);
        barometer_sample = barometer.Sample(state);
        magnetometer_sample = magnetometer.Sample(state);
    }
};

Simulation::Simulation(const AircraftDefinition &definition) : impl_(std::make_unique<Impl>(definition)) {}
Simulation::~Simulation() = default;

void Simulation::SelectController(int slot) {
    const auto controllers = impl_->definition.available_controllers;
    if (slot < 1 || static_cast<std::size_t>(slot) > controllers.size()) return;
    const auto selected = controllers[slot - 1];
    if (selected == impl_->active) return;
    impl_->active = selected;
    impl_->f450_demo.Reset();
    impl_->f450_angle.Reset();
    impl_->f450_horizon.Reset();
    impl_->f450_position_hold.Reset();
    impl_->c172_direct.Reset();
    impl_->commands.Clear();
}

void Simulation::Step(const ControllerKeys &keys) {
    const ControllerInput input{impl_->imu_sample, impl_->gps_sample, impl_->barometer_sample,
                                impl_->magnetometer_sample, static_cast<float>(GetPhysicsStepSeconds()), keys};
    impl_->commands.Clear();
    switch (impl_->active) {
    case ControllerType::F450Demo:
        impl_->f450_demo.Update(input, impl_->commands);
        break;
    case ControllerType::F450Angle:
        impl_->f450_angle.Update(input, impl_->commands);
        break;
    case ControllerType::F450Horizon:
        impl_->f450_horizon.Update(input, impl_->commands);
        break;
    case ControllerType::F450PositionHold:
        impl_->f450_position_hold.Update(input, impl_->commands);
        break;
    case ControllerType::C172Direct:
        impl_->c172_direct.Update(input, impl_->commands);
        break;
    }
    impl_->commands.Normalize();
    const auto previous_rpm = impl_->state.propulsion_rpm;
    impl_->backend.Step(impl_->commands);
    impl_->Refresh();
    for (std::size_t i = 0; i < impl_->rotor_angles.size(); ++i) {
        const double mean_rpm = (previous_rpm[i] + impl_->state.propulsion_rpm[i]) * 0.5;
        impl_->rotor_angles[i] = std::remainder(impl_->rotor_angles[i] +
            mean_rpm * 2 * std::numbers::pi / 60 * GetPhysicsStepSeconds(), 2 * std::numbers::pi);
    }
}

void Simulation::Reset() {
    impl_->backend.Reset(GetPhysicsStepSeconds());
    impl_->f450_demo.Reset();
    impl_->f450_angle.Reset();
    impl_->f450_horizon.Reset();
    impl_->f450_position_hold.Reset();
    impl_->c172_direct.Reset();
    impl_->commands.Clear();
    impl_->rotor_angles.fill(0);
    impl_->Refresh();
}

ControllerType Simulation::GetActiveController() const { return impl_->active; }
float Simulation::GetActiveControllerThrottle() const {
    if (impl_->active == ControllerType::C172Direct) return impl_->c172_direct.GetThrottle();
    if (impl_->active == ControllerType::F450Angle) return impl_->f450_angle.GetThrottle();
    if (impl_->active == ControllerType::F450Horizon) return impl_->f450_horizon.GetThrottle();
    return 0.0f;
}
const ControllerOutput &Simulation::GetLatestCommands() const { return impl_->commands; }
const AircraftDefinition &Simulation::GetAircraftDefinition() const { return impl_->definition; }
int Simulation::GetPhysicsFrequencyHz() const { return impl_->frequency_hz; }
double Simulation::GetPhysicsStepSeconds() const { return 1.0 / impl_->frequency_hz; }
void Simulation::SetPhysicsFrequencyHz(int hz) {
    impl_->frequency_hz = std::clamp(hz, 1, 240);
    impl_->backend.SetTimestep(GetPhysicsStepSeconds());
}
const AircraftState &Simulation::InspectAircraft() const { return impl_->state; }
glm::dvec3 Simulation::GetAircraftPosition() const { return DisplayBasis() * impl_->state.position_ned_ft; }
AircraftRenderState Simulation::GetAircraftRenderState() const {
    const auto basis = DisplayBasis();
    return {GetAircraftPosition(),
            glm::normalize(glm::quat_cast(basis * glm::mat3_cast(impl_->state.body_to_ned) * glm::transpose(basis))),
            impl_->state.timestamp_seconds, impl_->rotor_angles, impl_->state.left_aileron_rad,
            impl_->state.right_aileron_rad, impl_->state.elevator_rad, impl_->state.rudder_rad, impl_->state.flap_rad};
}
const ImuSample &Simulation::GetLatestImuSample() const { return impl_->imu_sample; }
const GpsSample &Simulation::GetLatestGpsSample() const { return impl_->gps_sample; }
const BarometerSample &Simulation::GetLatestBarometerSample() const { return impl_->barometer_sample; }
const MagnetometerSample &Simulation::GetLatestMagnetometerSample() const { return impl_->magnetometer_sample; }

int Simulation::RunSmokeTest() {
    const auto initial = impl_->state;
    auto check_state = [&] {
        const auto &s = impl_->state;
        return Finite(s.position_ned_ft) && Finite(s.velocity_ned_ft_per_second)
            && Finite(s.body_rates_rad_per_second) && Finite(s.body_specific_force_ft_per_second_squared)
            && std::isfinite(s.body_to_ned.w) && std::isfinite(s.body_to_ned.x)
            && std::isfinite(s.body_to_ned.y) && std::isfinite(s.body_to_ned.z)
            && s.pressure_psf > 0
            && impl_->imu_sample.timestamp_seconds == s.timestamp_seconds;
    };
    bool valid = check_state();
    for (int i = 0; i < 120; ++i) { Step({}); valid = valid && check_state(); }
    ControllerKeys keys;
    keys.r = true;
    if (impl_->definition.motor_properties.empty()) keys.d = keys.g = true;
    for (int i = 0; i < 30; ++i) { Step(keys); valid = valid && check_state(); }
    if (impl_->definition.motor_properties.empty()) {
        valid = valid && impl_->commands.throttle > 0 && impl_->commands.aileron == 1 && impl_->commands.flaps > 0;
    } else {
        valid = valid && impl_->commands.throttle == 0 && impl_->commands.aileron == 0
            && impl_->commands.elevator == 0 && impl_->commands.rudder == 0
            && impl_->commands.GetMotorCount() == impl_->definition.motor_properties.size()
            && impl_->commands.GetMotor(0) == 0;
        ControllerKeys arm;
        arm.x = true;
        Step(arm);
        valid = valid && impl_->commands.GetMotor(0) == 0.8f
            && impl_->commands.GetMotor(1) == 0.8f
            && impl_->commands.GetMotor(2) == 0.8f
            && impl_->commands.GetMotor(3) == 0.8f;
    }
    std::printf("%.*s: altitude=%.3f ft, throttle=%.3f, motor0=%.3f, rpm=%.3f, aileron=%.3f rad, flap=%.3f rad\n",
        static_cast<int>(impl_->definition.command_name.size()), impl_->definition.command_name.data(),
        impl_->state.altitude_asl_ft, impl_->commands.throttle, impl_->commands.GetMotor(0),
        impl_->state.propulsion_rpm[0], impl_->state.left_aileron_rad, impl_->state.flap_rad);
    SetPhysicsFrequencyHz(120);
    Reset();
    valid = valid && impl_->state.timestamp_seconds == 0 && impl_->commands.throttle == 0
        && glm::length(impl_->state.position_ned_ft - initial.position_ned_ft) < 1e-6;
    Step({});
    valid = valid && std::fabs(impl_->state.timestamp_seconds - 1.0 / 120) < 1e-9 && check_state();
    if (!valid) std::fprintf(stderr, "JSBSim backend smoke check failed\n");
    return valid ? EXIT_SUCCESS : EXIT_FAILURE;
}
