#include "trajectory_test.hpp"
#include "simulation.hpp"
#include "sensors/ideal_barometer.hpp"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <memory>
#include <stdexcept>
#include <string>

void RunTrajectoryTest(double mass_kg) {
    auto aircraft = GetAircraftDefinition(AircraftType::F450);
    aircraft.empty_mass_kg = mass_kg;
    Simulation sim(aircraft);
    const auto controllers = aircraft.available_controllers;
    const auto slot = std::ranges::find(controllers, ControllerType::F450PositionHold);
    if (slot == controllers.end()) throw std::runtime_error("F450 position hold unavailable");
    sim.SelectController(static_cast<int>(slot - controllers.begin()) + 1);
    sim.SetPhysicsFrequencyHz(60);

    const auto origin = sim.GetLatestGpsSample().position_ned_ft;
    const double initial_height = BarometricAltitudeFt(sim.GetLatestBarometerSample().pressure_psf);
    std::filesystem::create_directories("outputs");
    const auto prefix = "outputs/f450_position-hold_trajectory_" + std::to_string(mass_kg) + "kg_";
    std::string path;
    auto close_file = [](FILE *f) { std::fclose(f); };
    std::unique_ptr<FILE, decltype(close_file)> file(nullptr, close_file);
    for (unsigned id = 1; !file; ++id) {
        path = prefix + std::to_string(id) + ".csv";
        file.reset(std::fopen(path.c_str(), "wx"));
        if (!file && errno != EEXIST) throw std::runtime_error("Cannot open " + path);
    }
    std::fputs("tick,time_s,mass_kg,target_north_ft,target_east_ft,target_height_ft,north_ft,east_ft,height_ft,"
               "vertical_speed_fps,base_throttle,altitude_pid,collective,motor_fr,motor_al,motor_fl,motor_ar\n", file.get());

    size_t segment = 0;
    double squared_errors[BaselineTrajectory.size()]{};
    int samples[BaselineTrajectory.size()]{};
    double hover_throttle = 0, hover_speed = 0;
    for (int tick = 0; tick < 3600; ++tick) {
        if (segment + 1 < BaselineTrajectory.size() && tick == BaselineTrajectory[segment + 1].tick) ++segment;
        const auto [north, east, height] = BaselineTrajectory[segment].target;
        if (tick == BaselineTrajectory[segment].tick)
            sim.SetPositionTarget({origin.x + north, origin.y + east, initial_height + height});

        const auto gps = sim.GetLatestGpsSample();
        const double measured = BarometricAltitudeFt(sim.GetLatestBarometerSample().pressure_psf) - initial_height;
        const double speed = -gps.velocity_ned_ft_per_second.z;
        const double mass = sim.GetMassKg();
        sim.Step({});
        const auto altitude = sim.GetAltitudeCommands();
        const auto &motors = sim.GetLatestCommands();
        const double row[]{gps.timestamp_seconds, mass, north, east, height, gps.position_ned_ft.x - origin.x,
                           gps.position_ned_ft.y - origin.y, measured, speed, altitude.base, altitude.pid,
                           altitude.collective, motors.GetMotor(0), motors.GetMotor(1), motors.GetMotor(2), motors.GetMotor(3)};
        std::fprintf(file.get(), "%d", tick);
        for (const double value : row) {
            if (!std::isfinite(value)) throw std::runtime_error("Non-finite sample in " + path);
            std::fprintf(file.get(), ",%.17g", value);
        }
        std::fputc('\n', file.get());
        if (std::ferror(file.get())) throw std::runtime_error("Cannot write " + path);
        squared_errors[segment] += (height - measured) * (height - measured);
        ++samples[segment];
        if (tick >= 600 && tick < 900) {
            hover_throttle += altitude.collective;
            hover_speed += speed;
        }
    }
    if (std::fclose(file.release()) != 0) throw std::runtime_error("Cannot close " + path);
    std::printf("Trajectory CSV: %s\nSimulated mass: %.4f kg\n", path.c_str(), sim.GetMassKg());
    for (size_t i = 0; i < BaselineTrajectory.size(); ++i)
        std::printf("Segment %zu altitude RMSE: %.4f ft\n", i + 1, std::sqrt(squared_errors[i] / samples[i]));
    std::printf("Initial hover (10-15 s): mean collective %.5f, mean vertical speed %.4f ft/s\n",
                hover_throttle / 300, hover_speed / 300);
}
