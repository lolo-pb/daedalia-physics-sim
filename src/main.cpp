#include "app/application.hpp"
#include "app/simulation.hpp"

#include <cstdlib>
#include <string_view>

int main(int argument_count, char **arguments) {
    const bool run_smoke_test = argument_count == 2
        && std::string_view(arguments[1]) == "--smoke-test";

    InitializePhysicsRuntime();
    Simulation simulation;
    if (run_smoke_test) {
        return simulation.RunSmokeTest();
    }

    const int result = RunInteractiveApplication(simulation);

    if (result != EXIT_SUCCESS) {
        return result;
    }
    ShutdownPhysicsRuntime();
    return EXIT_SUCCESS;
}
