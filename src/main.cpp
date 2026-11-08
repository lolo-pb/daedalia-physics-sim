#include "app/application.hpp"
#include "app/simulation.hpp"
#include "aircraft/aircraft_definition.hpp"

#include <cstdio>
#include <cstdlib>
#include <exception>
#include <optional>
#include <string_view>

int main(int argc, char **argv) {
    std::optional<AircraftType> aircraft;
    bool smoke = false;
    for (int i = 1; i < argc; ++i) {
        const std::string_view argument(argv[i]);
        if (argument == "--help") {
            std::printf("Usage: %s [--aircraft f450|c172] [--smoke-test]\n", argv[0]);
            return EXIT_SUCCESS;
        }
        if (argument == "--smoke-test") { smoke = true; continue; }
        if (argument == "--aircraft" && i + 1 < argc) {
            aircraft = FindAircraftType(argv[++i]);
            if (aircraft) continue;
            std::fprintf(stderr, "Unknown aircraft: %s (available: f450, c172)\n", argv[i]);
            return EXIT_FAILURE;
        }
        std::fprintf(stderr, "Invalid argument: %s; use --help\n", argv[i]);
        return EXIT_FAILURE;
    }
    try {
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
