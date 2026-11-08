#include "altitude_test.hpp"
#include "simulation.hpp"

#include <cerrno>
#include <charconv>
#include <limits>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <memory>
#include <stdexcept>
#include <string>

namespace {
// Quote each argument for the POSIX shell used by std::system.
std::string ShellQuote(std::string_view value) {
	std::string quoted = "'";
	for (char character : value) {
		if (character == '\'')
			quoted += "'\\''";
		else
			quoted += character;
	}
	return quoted + "'";
}

std::string_view ControllerName(ControllerType type) {
	switch (type) {
	case ControllerType::F450Demo:
		return "demo";
	case ControllerType::F450Angle:
		return "angle";
	case ControllerType::F450Horizon:
		return "horizon";
	case ControllerType::F450PositionHold:
		return "position-hold";
	case ControllerType::C172Direct:
		return "direct";
	}
	return {};
}
} // namespace

void RunAltitudeTest(const AircraftDefinition &aircraft, std::string_view controller, double duration_seconds, bool plot) {
	int slot = 0;
	for (std::size_t i = 0; i < aircraft.available_controllers.size(); ++i) {
		if (ControllerName(aircraft.available_controllers[i]) == controller) slot = static_cast<int>(i) + 1;
	}
	if (!slot) throw std::invalid_argument("Controller unavailable for this aircraft; use --help");

	const double steps = std::floor(duration_seconds * 60);
	if (!std::isfinite(steps) || steps >= 18446744073709551616.0)
		throw std::invalid_argument("Duration is too large");
	Simulation simulation(aircraft);
	simulation.SelectController(slot);
	std::filesystem::create_directories("outputs");
	const auto prefix = std::string(aircraft.command_name) + "_" + std::string(controller) + "_";
	std::uint64_t highest = 0;
	for (const auto &entry : std::filesystem::directory_iterator("outputs")) {
		const auto extension = entry.path().extension();
		if (extension != ".csv" && extension != ".png") continue;
		const auto stem = entry.path().stem().string();
		if (!stem.starts_with(prefix)) continue;
		const std::string_view number(stem.data() + prefix.size(), stem.size() - prefix.size());
		std::uint64_t value = 0;
		const auto parsed = std::from_chars(number.data(), number.data() + number.size(), value);
		if (parsed.ec == std::errc{} && parsed.ptr == number.data() + number.size())
			highest = std::max(highest, value);
	}
	std::string path;
	auto close_file = [](FILE *handle) { std::fclose(handle); };
	std::unique_ptr<FILE, decltype(close_file)> file(nullptr, close_file);
	while (!file) {
		if (highest == std::numeric_limits<std::uint64_t>::max())
			throw std::runtime_error("Output numbering exhausted");
		const auto base = "outputs/" + prefix + std::to_string(++highest);
		if (std::filesystem::exists(base + ".png")) continue;
		path = base + ".csv";
		file.reset(std::fopen(path.c_str(), "wx"));
		if (!file && errno != EEXIST) throw std::runtime_error("Cannot create CSV: " + path);
	}
	const double initial_altitude = simulation.InspectAircraft().altitude_asl_ft;
	auto write_sample = [&] {
		const auto &state = simulation.InspectAircraft();
		const double height = state.altitude_asl_ft - initial_altitude;
		if (!std::isfinite(state.timestamp_seconds) || !std::isfinite(height))
			throw std::runtime_error("Non-finite altitude sample; partial CSV: " + path);
		if (std::fprintf(file.get(), "%.17g,%.17g\n", state.timestamp_seconds, height) < 0)
			throw std::runtime_error("Cannot write CSV: " + path);
	};
	if (std::fprintf(file.get(), "time_seconds,height_change_ft\n") < 0)
		throw std::runtime_error("Cannot write CSV: " + path);
	write_sample();
	for (std::uint64_t tick = 0; tick < static_cast<std::uint64_t>(steps); ++tick) {
		simulation.Step({});
		write_sample();
	}
	if (std::fclose(file.release()) != 0) throw std::runtime_error("Cannot close CSV: " + path);
	std::printf("Altitude CSV: %s\n", path.c_str());
	if (plot) {
		std::fflush(stdout);
		printf("Ploting...\n");
		const auto command = ShellQuote(DAEDALIA_PYTHON_EXECUTABLE) + " " + ShellQuote(DAEDALIA_PLOT_SCRIPT) + " " + ShellQuote(path);
		if (std::system(command.c_str()) != 0)
			throw std::runtime_error("Plot generation failed; CSV preserved: " + path);
		printf("Done\n");
	}
}
