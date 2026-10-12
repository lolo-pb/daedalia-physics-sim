#include "ui.hpp"
#include <imgui.h>
#include "simulation.hpp"

PhysicsPanelResult DrawPhysicsPanel(Simulation &simulation, bool &paused,
                                    bool &single_step, bool &follow_drone) {
    PhysicsPanelResult result;
    ImGui::Begin("Physics");
    const auto &definition = simulation.GetAircraftDefinition();
    ImGui::TextUnformatted(definition.display_name.data());
    ImGui::SeparatorText("Controller");
    for (std::size_t i = 0; i < definition.available_controllers.size(); ++i) {
        if (ImGui::RadioButton("1 Manual", simulation.GetActiveController() == definition.available_controllers[i])) {
            simulation.SelectController(static_cast<int>(i + 1));
        }
    }
    ImGui::Text("Throttle: %.3f", simulation.GetActiveControllerThrottle());
    ImGui::TextUnformatted("W/S pitch, A/D roll, Q/E yaw, R/F throttle");
    if (!definition.motor_properties.empty()) ImGui::TextUnformatted("Raw motor mix; no simulator stabilization");
    ImGui::TextUnformatted("G/H flaps, hold B for brakes");
    ImGui::TextUnformatted("Hold right mouse for camera controls");
    ImGui::SeparatorText("Simulation");
    if (ImGui::Button(paused ? "Resume" : "Pause")) paused = !paused;
    ImGui::SameLine();
    if (ImGui::Button("Single step")) { paused = true; single_step = true; }
    ImGui::SameLine();
    if (ImGui::Button("Reset")) { simulation.Reset(); result.reset = true; }
    int hz = simulation.GetPhysicsFrequencyHz();
    if (ImGui::SliderInt("Physics frequency (Hz)", &hz, 1, 240)) {
        simulation.SetPhysicsFrequencyHz(hz);
        result.physics_frequency_changed = true;
    }
    if (ImGui::Checkbox("Follow aircraft", &follow_drone) && follow_drone) result.follow_drone_enabled = true;
    const auto &state = simulation.InspectAircraft();
    ImGui::SeparatorText("JSBSim state (native units)");
    ImGui::Text("Time: %.3f s", state.timestamp_seconds);
    ImGui::Text("Position NED (ft): %.3f, %.3f, %.3f", state.position_ned_ft.x, state.position_ned_ft.y, state.position_ned_ft.z);
    ImGui::Text("Velocity NED (ft/s): %.3f, %.3f, %.3f", state.velocity_ned_ft_per_second.x, state.velocity_ned_ft_per_second.y, state.velocity_ned_ft_per_second.z);
    ImGui::Text("Mass: %.3f slugs", state.mass_slugs);
    ImGui::Text("Gravity: %.3f ft/s^2", state.gravity_ft_per_second_squared);
    ImGui::Text("Altitude ASL: %.3f ft", state.altitude_asl_ft);
    const auto &commands = simulation.GetLatestCommands();
    ImGui::Text("Roll/pitch/yaw commands: %.2f, %.2f, %.2f", commands.roll, commands.pitch, commands.yaw);
    ImGui::Text("Flaps: %.2f  Brakes: %.2f / %.2f", commands.flaps, commands.left_brake, commands.right_brake);
    ImGui::End();
    return result;
}

void DrawSensorsPanel(const Simulation &simulation) {
    const auto &imu = simulation.GetLatestImuSample();
    const auto &gps = simulation.GetLatestGpsSample();
    const auto &barometer = simulation.GetLatestBarometerSample();
    const auto &magnetometer = simulation.GetLatestMagnetometerSample();
    ImGui::Begin("Sensors");
    ImGui::Text("Sample time: %.3f s", imu.timestamp_seconds);
    ImGui::SeparatorText("IMU (body: forward, right, down)");
    const auto &gyro = imu.body_gyro_rad_per_second;
    const auto &force = imu.body_specific_force_ft_per_second_squared;
    ImGui::Text("Gyro (rad/s): %.3f, %.3f, %.3f", gyro.x, gyro.y, gyro.z);
    ImGui::Text("Specific force (ft/s^2): %.3f, %.3f, %.3f", force.x, force.y, force.z);
    ImGui::SeparatorText("GPS (north, east, down)");
    const auto &position = gps.position_ned_ft;
    const auto &velocity = gps.velocity_ned_ft_per_second;
    ImGui::Text("Position (ft): %.3f, %.3f, %.3f", position.x, position.y, position.z);
    ImGui::Text("Velocity (ft/s): %.3f, %.3f, %.3f", velocity.x, velocity.y, velocity.z);
    ImGui::SeparatorText("Barometer");
    ImGui::Text("Pressure: %.3f psf", barometer.pressure_psf);
    ImGui::SeparatorText("Magnetometer");
    const auto &field = magnetometer.body_magnetic_field_microteslas;
    ImGui::Text("Field (body, uT): %.3f, %.3f, %.3f", field.x, field.y, field.z);
    ImGui::End();
}
