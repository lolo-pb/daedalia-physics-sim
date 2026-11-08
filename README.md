# Daedalia Physics Sim

Daedalia is a native C++20 flight-simulation sandbox for programmable sensors and flight controllers. JSBSim provides the aircraft dynamics and ground reactions; SDL3, OpenGL, and ImGui provide the interactive interface.

The current aircraft are an F450 quadcopter and a Cessna 172, both backed by bundled JSBSim models and rendered as simple representative geometry driven by live simulation state.

## Build and run

Requires CMake 3.24+, a C++20 compiler, Python 3, and OpenGL development libraries. CMake downloads JSBSim, SDL3, GLM, glad, and ImGui during the first configuration.

```sh
cmake -S . -B build
cmake --build build -j4
./build/daedalia
```

Choose an aircraft in the startup menu, or launch one directly:

```sh
./build/daedalia --aircraft f450
./build/daedalia --aircraft c172
```

JSBSim aircraft, engine, and system data are copied to `build/jsbsim-data` during configuration. Keep that directory with the executable.

To give an editor the exact CMake include paths:

```sh
cmake -S . -B build -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```

This writes `build/compile_commands.json`.

## Runtime

The startup menu selects an aircraft. The Physics panel selects one of that aircraft's available controllers:

- F450: demo, angle, horizon, and position hold
- C172: direct

The application supports keyboard controller input, a right-mouse camera, pause, single step, reset, aircraft follow, and configurable simulation frequency. The simulation/controller/sensor tick defaults to 60 Hz and can be set from 1 to 240 Hz. JSBSim uses smaller internal integration steps where required by the aircraft definition.

## Architecture

`src/aircraft/` defines each aircraft, including its JSBSim model, startup conditions, actuator bindings, visual parts, and selectable controllers.

`src/physics/` owns JSBSim and native simulation state. 

`src/sensors/` publishes programmable sensor samples. 

`src/controllers/` owns the shared `ControllerInput` / `ControllerOutput` contract. 

`src/app/` owns the fixed-step loop, UI, camera, and OpenGL renderer.

Controllers receive sensor samples rather than `AircraftState`. Simulation state remains available to the backend, sensors, renderer, and debug UI.

## Units and frames

Controller-facing values retain JSBSim-native units:

- Position: feet
- Velocity: feet per second
- Specific force: feet per second squared
- Pressure: pounds per square foot
- Angular rate: radians per second

Body axes are forward, right, down. World position and velocity use local north, east, down (NED). The renderer converts those frames into its right, up, back display frame.

The supplied IMU, GPS, barometer, and magnetometer models are ideal at present: they have no modeled noise, latency, bias, quantization, dropout, or GPS error.

## Checks

```sh
./build/daedalia --smoke-test --aircraft f450
./build/daedalia --smoke-test --aircraft c172
ctest --test-dir build --output-on-failure
```

The smoke checks cover model loading, stepping, finite state, sensor publication, basic controller commands, reset, and frequency changes. They do not validate controller tuning or flight stability.

## Experimental geometry drones

One of the defining features of this sim is the capability of defining custom geometry for experimental rotor craft. 

The coolest one so far has been the triangular bipyramid drone, a custom-aircraft concept based on a 6 face triangular bipyramid geometry, for wich it used all its faces to maintain altitude at any orientation. It is maintained on a separate branch. It is not part of the aircraft list in this branch yet.
