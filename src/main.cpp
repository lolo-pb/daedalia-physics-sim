#include "app/application.hpp"
#include "app/simulation.hpp"
#include "drones/drone_definition.hpp"

#include <cstdio>
#include <cstdlib>
#include <optional>
#include <string_view>

namespace {

struct ProgramOptions {
    bool run_smoke_test = false;
    bool arguments_are_valid = true;
    std::optional<DroneType> selected_drone;
};

void PrintUsage(const char *program_name) {
    std::printf(
        "Usage: %s [--drone <name>] [--smoke-test]\n"
        "       %s --help\n\n"
        "Available drones:\n",
        program_name,
        program_name);
    for (const DroneOption &option : GetAvailableDroneOptions()) {
        std::printf("  %.*s\n",
            static_cast<int>(option.command_name.size()),
            option.command_name.data());
    }
}

bool HasHelpFlag(int argument_count, char **arguments) {
    for (int index = 1; index < argument_count; ++index) {
        if (std::string_view(arguments[index]) == "--help") {
            return true;
        }
    }
    return false;
}

ProgramOptions ParseOptions(int argument_count, char **arguments) {
    ProgramOptions options;
    for (int index = 1; index < argument_count; ++index) {
        const std::string_view argument(arguments[index]);
        if (argument == "--smoke-test") {
            options.run_smoke_test = true;
            continue;
        }
        if (argument == "--drone") {
            if (index + 1 >= argument_count
                || std::string_view(arguments[index + 1]).starts_with("--")) {
                std::fprintf(stderr, "Warning: --drone requires a name\n");
                options.arguments_are_valid = false;
                continue;
            }
            const std::string_view drone_name(arguments[++index]);
            const std::optional<DroneType> drone_type =
                FindAvailableDroneType(drone_name);
            if (!drone_type) {
                std::fprintf(
                    stderr,
                    "Warning: drone '%.*s' is not available\n",
                    static_cast<int>(drone_name.size()),
                    drone_name.data());
                options.arguments_are_valid = false;
                continue;
            }
            options.selected_drone = drone_type;
            continue;
        }

        std::fprintf(
            stderr,
            "Warning: unknown argument '%.*s'\n",
            static_cast<int>(argument.size()),
            argument.data());
        options.arguments_are_valid = false;
    }

    if (!options.arguments_are_valid) {
        options.selected_drone.reset();
    }
    return options;
}

} // namespace

int main(int argument_count, char **arguments) {
    if (HasHelpFlag(argument_count, arguments)) {
        PrintUsage(arguments[0]);
        return EXIT_SUCCESS;
    }

    const ProgramOptions options = ParseOptions(argument_count, arguments);
    if (!options.arguments_are_valid) {
        std::fprintf(
            stderr,
            options.run_smoke_test
                ? "Warning: using the default quadcopter for the smoke test\n"
                : "Warning: opening the drone selection menu\n");
    }

    PhysicsRuntime physics_runtime;
    if (options.run_smoke_test) {
        const DroneType drone_type =
            options.selected_drone.value_or(DefaultDroneType);
        Simulation simulation(CreateDroneDefinition(drone_type));
        int smoke_test_result = simulation.RunSmokeTest();
        if (smoke_test_result == EXIT_SUCCESS
            && drone_type == DroneType::Quadcopter) {
            smoke_test_result = simulation.RunQuadcopterControlSmokeTest();
        }
        if (smoke_test_result == EXIT_SUCCESS) {
            std::printf("Simulation smoke test passed\n");
        }
        return smoke_test_result;
    }
    return RunInteractiveApplication(options.selected_drone);
}
