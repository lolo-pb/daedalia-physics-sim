#include "app/application.hpp"
#include "app/simulation.hpp"

#include <string_view>

int main(int argument_count, char **arguments) {
    const bool run_smoke_test = argument_count == 2
        && std::string_view(arguments[1]) == "--smoke-test";

    PhysicsRuntime physics_runtime;
    Simulation simulation;
    if (run_smoke_test) {
        return simulation.RunSmokeTest();
    }
    return RunInteractiveApplication(simulation);
}
