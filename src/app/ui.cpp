#include "ui.hpp"

#include <array>

#include <imgui.h>

#include "sensors/ideal_barometer.hpp"
#include "simulation.hpp"

PhysicsPanelResult DrawPhysicsPanel(
    Simulation &simulation,
    bool &paused,
    bool &single_step,
    bool &follow_drone) {
    PhysicsPanelResult result;

    ImGui::Begin("Physics");
    ImGui::SeparatorText("Flight controller");
    if (ImGui::RadioButton("1 Demo", simulation.GetActiveController() == FlightController::Demo)) {
        simulation.SelectController(1);
    }
    ImGui::SameLine();
    if (ImGui::RadioButton("2 Manual Hover", simulation.GetActiveController() == FlightController::ManualHover)) {
        simulation.SelectController(2);
    }
    ImGui::SameLine();
    if (ImGui::RadioButton("3 Position Hold", simulation.GetActiveController() == FlightController::PositionHold)) {
        simulation.SelectController(3);
    }
    if (simulation.GetActiveController() == FlightController::ManualHover) {
        ImGui::Text("Throttle: %.3f", simulation.GetManualControllerThrottle());
        ImGui::TextUnformatted("W/S pitch, A/D roll, Q/E yaw, R/F throttle");
        ImGui::TextUnformatted("Hold right mouse for camera controls");
    }
    if (simulation.GetActiveController() == FlightController::PositionHold) {
        ImGui::TextUnformatted("W/S forward, A/D right, Q/E yaw, R/F altitude");
        ImGui::TextUnformatted("Commands move the held target relative to its heading");
        ImGui::TextUnformatted("Hold right mouse for camera controls");
    }

    ImGui::SeparatorText("Simulation");
    if (ImGui::Button(paused ? "Resume" : "Pause")) {
        paused = !paused;
    }
    ImGui::SameLine();
    if (ImGui::Button("Single step")) {
        paused = true;
        single_step = true;
    }
    ImGui::SameLine();
    if (ImGui::Button("Reset")) {
        simulation.Reset();
        result.reset = true;
    }

    std::array<float, 3> gravity = simulation.GetGravity();
    if (ImGui::DragFloat3("Gravity", gravity.data(), 0.1f)) {
        simulation.SetGravity(gravity);
    }

    int physics_frequency_hz = simulation.GetPhysicsFrequencyHz();
    if (ImGui::SliderInt("Physics frequency (Hz)", &physics_frequency_hz, 1, 120)) {
        simulation.SetPhysicsFrequencyHz(physics_frequency_hz);
        result.physics_frequency_changed = true;
    }

    ImGui::SeparatorText("Camera");
    if (ImGui::Checkbox("Follow drone", &follow_drone) && follow_drone) {
        result.follow_drone_enabled = true;
    }

    const DroneInspection drone = simulation.InspectDrone();
    ImGui::SeparatorText("Drone");
    ImGui::Text("Position: %.3f, %.3f, %.3f", drone.position.GetX(), drone.position.GetY(), drone.position.GetZ());
    ImGui::Text("Rotation: %.3f, %.3f, %.3f, %.3f", drone.rotation.GetX(), drone.rotation.GetY(), drone.rotation.GetZ(), drone.rotation.GetW());
    ImGui::Text("Linear velocity: %.3f, %.3f, %.3f", drone.linear_velocity.GetX(), drone.linear_velocity.GetY(), drone.linear_velocity.GetZ());
    ImGui::Text("Angular velocity: %.3f, %.3f, %.3f", drone.angular_velocity.GetX(), drone.angular_velocity.GetY(), drone.angular_velocity.GetZ());
    ImGui::Text("Mass: %.3f", drone.mass);
    ImGui::End();

    return result;
}

void DrawSensorsPanel(const Simulation &simulation) {
    const ImuSample &imu = simulation.GetLatestImuSample();
    const GpsSample &gps = simulation.GetLatestGpsSample();
    const BarometerSample &barometer = simulation.GetLatestBarometerSample();
    const MagnetometerSample &magnetometer = simulation.GetLatestMagnetometerSample();

    ImGui::Begin("Sensors");
    ImGui::SeparatorText("IMU");
    ImGui::Text("Timestamp: %.3f s", imu.timestamp_seconds);
    ImGui::Text(
        "Gyro (body, rad/s): %.3f, %.3f, %.3f",
        imu.body_gyro_rad_per_second.x,
        imu.body_gyro_rad_per_second.y,
        imu.body_gyro_rad_per_second.z);
    ImGui::Text(
        "Specific force (body, m/s^2): %.3f, %.3f, %.3f",
        imu.body_specific_force_meters_per_second_squared.x,
        imu.body_specific_force_meters_per_second_squared.y,
        imu.body_specific_force_meters_per_second_squared.z);
    ImGui::SeparatorText("GPS");
    ImGui::Text(
        "Position (world, m): %.3f, %.3f, %.3f",
        gps.world_position_meters.x,
        gps.world_position_meters.y,
        gps.world_position_meters.z);
    ImGui::Text(
        "Velocity (world, m/s): %.3f, %.3f, %.3f",
        gps.world_velocity_meters_per_second.x,
        gps.world_velocity_meters_per_second.y,
        gps.world_velocity_meters_per_second.z);
    ImGui::SeparatorText("Barometer");
    ImGui::Text("Pressure: %.2f Pa", barometer.pressure_pascals);
    ImGui::Text("Altitude: %.3f m", BarometricAltitudeMeters(barometer.pressure_pascals));
    ImGui::SeparatorText("Magnetometer");
    ImGui::Text(
        "Magnetic field (body, uT): %.3f, %.3f, %.3f",
        magnetometer.body_magnetic_field_microteslas.x,
        magnetometer.body_magnetic_field_microteslas.y,
        magnetometer.body_magnetic_field_microteslas.z);
    ImGui::End();
}
