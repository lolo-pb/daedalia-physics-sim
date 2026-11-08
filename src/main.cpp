#include "aircraft/aircraft_definition.hpp"
#include "app/application.hpp"
#include "app/simulation.hpp"

#include "app/altitude_test.hpp"

#include <charconv>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <optional>
#include <string_view>

int main(int argc, char **argv) {
	std::optional<AircraftType> aircraft;
	bool smoke = false;
	bool plot = false;
	std::optional<AircraftType> test_aircraft;
	std::string_view test_controller;
	double duration = 0;
	for (int i = 1; i < argc; ++i) {
		const std::string_view argument(argv[i]);
		if (argument == "--help") {
			std::printf("Usage: %s [--aircraft f450|c172] [--smoke-test]\n", argv[0]);
			std::printf("       %s --altitude-test <aircraft> <controller> <seconds> [--plot]\n"
									"Controllers: f450: demo, angle, horizon, position-hold; c172: direct\n",
									argv[0]);
			return EXIT_SUCCESS;
		}
		if (argument == "--altitude-test" && i + 3 < argc && !test_aircraft) {
			test_aircraft = FindAircraftType(argv[++i]);
			test_controller = argv[++i];
			const std::string_view seconds(argv[++i]);
			const auto parsed = std::from_chars(seconds.data(), seconds.data() + seconds.size(), duration);
			if (!test_aircraft || parsed.ec != std::errc{} || parsed.ptr != seconds.data() + seconds.size() || !std::isfinite(duration) || duration <= 0) {
				std::fprintf(stderr, "Invalid altitude test aircraft or duration; use --help\n");
				return EXIT_FAILURE;
			}
			continue;
		}
		if (argument == "--plot") {
			plot = true;
			continue;
		}
		if (argument == "--smoke-test") {
			smoke = true;
			continue;
		}
		if (argument == "--aircraft" && i + 1 < argc) {
			aircraft = FindAircraftType(argv[++i]);
			if (aircraft) continue;
			std::fprintf(stderr, "Unknown aircraft: %s (available: f450, c172)\n", argv[i]);
			return EXIT_FAILURE;
		}
		std::fprintf(stderr, "Invalid argument: %s; use --help\n", argv[i]);
		return EXIT_FAILURE;
	}
	if (plot && !test_aircraft) {
		std::fprintf(stderr, "--plot requires --altitude-test\n");
		return EXIT_FAILURE;
	}
	try {
		if (test_aircraft) {
			if (smoke || aircraft) {
				std::fprintf(stderr, "--altitude-test cannot be combined with --aircraft or --smoke-test\n");
				return EXIT_FAILURE;
			}
			printf("Comencing test...\n");
			RunAltitudeTest(GetAircraftDefinition(*test_aircraft), test_controller, duration, plot);
			return EXIT_SUCCESS;
		}
		if (smoke) {
			Simulation simulation(GetAircraftDefinition(aircraft.value_or(DefaultAircraftType)));
			const int result = simulation.RunSmokeTest();
			if (result == EXIT_SUCCESS) std::printf("JSBSim backend smoke check passed\n");
			return result;
		}
		return RunInteractiveApplication(aircraft);
	} catch (const std::exception &error) {
		std::fprintf(stderr, "Daedalia: %s\n", error.what());
		return EXIT_FAILURE;
	}
}
