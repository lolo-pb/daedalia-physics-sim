#include "physics/jsbsim_backend.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <stdexcept>
#include <string>

#include <FGFDMExec.h>
#include <initialization/FGInitialCondition.h>
#include <models/FGAccelerations.h>
#include <models/FGAtmosphere.h>
#include <models/FGFCS.h>
#include <models/FGPropagate.h>
#include <models/FGPropulsion.h>
#include <input_output/FGPropertyManager.h>
#include <glm/gtc/quaternion.hpp>

namespace {
glm::dvec3 Vector(const JSBSim::FGColumnVector3 &v) { return {v(1), v(2), v(3)}; }
}

struct JsbsimBackend::Impl {
    JSBSim::FGFDMExec exec;
    JSBSim::FGLocation origin;
    int integration_steps = 1;

    explicit Impl(const AircraftDefinition &definition, double timestep) {
        exec.SetDebugLevel(0);
        const std::filesystem::path data(DAEDALIA_JSBSIM_DATA_DIR);
        if (!std::filesystem::exists(data / "aircraft" / definition.jsbsim_model)) {
            throw std::runtime_error("JSBSim model data missing: " + data.string());
        }
        exec.SetRootDir(SGPath(data.string()));
        exec.SetAircraftPath(SGPath("aircraft"));
        exec.SetEnginePath(SGPath("engine"));
        exec.SetSystemsPath(SGPath("systems"));
        // Stock model output declarations may open a header-only log during
        // initialization, even when runtime logging is disabled. Keep it in build data.
        exec.SetOutputPath(SGPath(data.string()));
        // Small aircraft ground reactions and electric propulsion need a smaller
        // integration interval than the user-facing controller/simulation tick.
        integration_steps = std::max(1, static_cast<int>(std::ceil(timestep / definition.max_integration_step_seconds)));
        exec.Setdt(timestep / integration_steps);
        if (!exec.LoadModel(std::string(definition.jsbsim_model))) {
            throw std::runtime_error("Cannot load JSBSim aircraft: " + std::string(definition.jsbsim_model));
        }
        exec.DisableOutput();
        const auto ic = exec.GetIC();
        ic->SetLatitudeDegIC(definition.latitude_deg);
        ic->SetLongitudeDegIC(definition.longitude_deg);
        ic->SetTerrainElevationFtIC(0);
        ic->SetAltitudeAGLFtIC(definition.initial_altitude_agl_ft);
        ic->SetPsiDegIC(0);
        if (!exec.RunIC()) throw std::runtime_error("JSBSim initial conditions failed");
        origin = exec.GetPropagate()->GetLocation();
        origin.SetPositionGeodetic(origin.GetLongitude(), origin.GetGeodLatitudeRad(), 0);
        for (const auto property : definition.control_properties) {
            if (!exec.GetPropertyManager()->HasNode(std::string(property))) {
                throw std::runtime_error("JSBSim control property missing: " + std::string(property));
            }
        }
        for (const auto property : definition.motor_properties) {
            if (!exec.GetPropertyManager()->HasNode(std::string(property))) {
                throw std::runtime_error("JSBSim motor property missing: " + std::string(property));
            }
        }
        for (const auto property : definition.simulator_control_disable_properties) {
            if (!exec.GetPropertyManager()->HasNode(std::string(property))) {
                throw std::runtime_error("JSBSim simulator-control property missing: " + std::string(property));
            }
            exec.SetPropertyValue(std::string(property), 0);
        }
        // Piston engines need starting. Electric motors run from throttle directly;
        // JSBSim's 0.5-second steady-state starter step is unsuitable for the F450.
        if (definition.initialize_running) exec.GetPropulsion()->InitRunning(-1);
        exec.GetFCS()->SetThrottleCmd(-1, 0);
        exec.GetFCS()->SetThrottlePos(-1, 0);
    }
};

JsbsimBackend::JsbsimBackend(const AircraftDefinition &definition) : definition_(definition) { Reset(1.0 / 60); }
JsbsimBackend::~JsbsimBackend() = default;
void JsbsimBackend::Reset(double timestep) { impl_ = std::make_unique<Impl>(definition_, timestep); }
void JsbsimBackend::SetTimestep(double seconds) {
    impl_->integration_steps = std::max(1, static_cast<int>(std::ceil(seconds / definition_.max_integration_step_seconds)));
    impl_->exec.Setdt(seconds / impl_->integration_steps);
}

void JsbsimBackend::Step(const ControllerOutput &commands) {
    const std::array<double, 7> values{commands.throttle, commands.aileron, commands.elevator, commands.rudder,
                                      commands.flaps, commands.left_brake, commands.right_brake};
    for (std::size_t i = 0; i < values.size(); ++i) {
        impl_->exec.SetPropertyValue(std::string(definition_.control_properties[i]), values[i]);
    }
    for (std::size_t i = 0; i < definition_.motor_properties.size(); ++i) {
        impl_->exec.SetPropertyValue(std::string(definition_.motor_properties[i]), commands.GetMotor(i));
    }
    for (int i = 0; i < impl_->integration_steps; ++i) {
        if (!impl_->exec.Run()) throw std::runtime_error("JSBSim simulation step failed");
    }
}

AircraftState JsbsimBackend::ReadState() const {
    auto &exec = impl_->exec;
    const auto propagate = exec.GetPropagate();
    AircraftState state;
    state.timestamp_seconds = exec.GetSimTime();
    const auto location = propagate->GetLocation();
    state.position_ned_ft = Vector(impl_->origin.GetTec2l() * (location - impl_->origin));
    const auto local_to_origin = impl_->origin.GetTec2l() * location.GetTl2ec();
    state.velocity_ned_ft_per_second = Vector(local_to_origin * propagate->GetVel());
    state.body_rates_rad_per_second = Vector(propagate->GetPQR());
    state.body_specific_force_ft_per_second_squared = Vector(exec.GetAccelerations()->GetBodyAccel());
    const auto matrix = local_to_origin * propagate->GetTb2l();
    glm::dmat3 rotation(1);
    for (int row = 0; row < 3; ++row)
        for (int column = 0; column < 3; ++column) rotation[column][row] = matrix(row + 1, column + 1);
    state.body_to_ned = glm::normalize(glm::quat_cast(rotation));
    state.pressure_psf = exec.GetAtmosphere()->GetPressure();
    state.altitude_asl_ft = propagate->GetAltitudeASL();
    for (unsigned i = 0; i < exec.GetPropulsion()->GetNumEngines() && i < state.propulsion_rpm.size(); ++i) {
        state.propulsion_rpm[i] = exec.GetPropertyValue("propulsion/engine[" + std::to_string(i) + "]/propeller-rpm");
    }
    state.left_aileron_rad = exec.GetPropertyValue("fcs/left-aileron-pos-rad");
    state.right_aileron_rad = exec.GetPropertyValue("fcs/right-aileron-pos-rad");
    state.elevator_rad = exec.GetPropertyValue("fcs/elevator-pos-rad");
    state.rudder_rad = exec.GetPropertyValue("fcs/rudder-pos-rad");
    state.flap_rad = exec.GetPropertyValue("fcs/flap-pos-rad");
    return state;
}
