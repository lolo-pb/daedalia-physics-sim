# Daedalia Physics Sim

A native C++ test bed for programmable sensors and flight controllers, powered by JSBSim 1.3.2. SDL creates the window, OpenGL draws representative aircraft, and ImGui displays simulation and sensor readings.

## Build and run

Requires a C++20 compiler, CMake 3.24+, Python 3 and OpenGL development libraries. CMake downloads JSBSim, SDL, GLM, glad and ImGui on first configuration.

```sh
cmake -S . -B build
cmake --build build -j4
./build/daedalia
./build/daedalia --aircraft f450
./build/daedalia --aircraft c172
```

Without an aircraft flag, choose the F450 quadcopter or Cessna 172 in the startup menu. The physics models are bundled JSBSim aircraft, not replicas of the old custom drones. The F450 is the default for headless checks.

Aircraft XML, engines and systems are copied into `build/jsbsim-data`. The executable uses an absolute configured data path, so launching from another working directory works. Keep that data directory with the build.

## Controls

Each aircraft declares its own controller. The F450 demo controller writes four raw motor commands (no mixer or stabilization); the C172 direct controller writes throttle and named actuator commands for aileron, elevator, rudder, flaps, and brakes. JSBSim provides aircraft dynamics only; its bundled stabilizers and autopilots are held off.

- W/S pitch, A/D roll, Q/E yaw.
- R/F increase/decrease throttle (starts at zero).
- G/H extend/retract flaps; hold B for brakes.
- Hold the right mouse button to operate the camera. W/A/S/D move it, Space/Ctrl move vertically, and Shift moves faster.
- Pause, single-step, reset, camera follow and physics frequency are available in the Physics panel.

The default simulation/controller/sensor tick is 60 Hz and is configurable from 1 to 240 Hz. JSBSim uses smaller internal integration steps where needed (at most 0.002 seconds for F450 and 1/120 second for C172, following the bundled example scripts). Commands are held through these substeps; sensors publish at the configured tick. Reset restores the on-ground starting pose, ready propulsion, zero commands, sensor readings and controller state.

## Units and graphics

Controller-facing samples use explicitly named native values: feet, feet/second, feet/second squared, pounds-force/square-foot and radians/second. Body axes are forward/right/down; GPS uses local north/east/down. Controllers own conversions needed by their algorithms. The ideal magnetic field is 50 microteslas pointing north.

The OpenGL display also uses feet. It maps NED/body axes to X right, Y up, -Z forward for the camera and geometry. The graphics are simple representative parts, with aircraft pose, rotor speeds and hinged surfaces driven by JSBSim state. Ground reactions come entirely from JSBSim; there is no general mesh collision system.

The stock F450 XML currently emits a configuration-version warning and duplicate internal sensor-property binding warnings. Backend stepping and lift-off were checked, but this is not a claim that the supplied model is fully validated. Daedalia's programmable sensors read JSBSim ground truth independently of those internal model sensors.

## Extension points

`src/aircraft/` contains a single `AircraftDefinition` per aircraft. It references the JSBSim model, starting conditions, geometry, command properties and available controllers. Geometry and controller lists can live in referenced files without splitting the aircraft declaration into independent catalogs.

`src/physics/` owns JSBSim and native ground truth. `src/sensors/` turns that state into programmable samples. `src/controllers/` defines the unified `ControllerInput` / `ControllerOutput` contract and the manual controller. `src/app/` owns the window, fixed-step loop, UI and rendering.

Controllers receive sensor samples, not the ground-truth object. Ground truth remains available to the backend, sensors, renderer and debug UI.

Existing controller implementations remain in the repository but are excluded from the build and selection. Their sensor names, units, motor mapping and tuning await individual review. F450's indexed motor commands are bound directly to its external ESC inputs; other aircraft can expose their own actuator bindings through AircraftDefinition.

## Focused backend checks

```sh
./build/daedalia --smoke-test --aircraft f450
./build/daedalia --smoke-test --aircraft c172
ctest --test-dir build --output-on-failure
```

These check model loading, stepping, finite state, sensor publication, manual commands and reset/frequency changes. They do not assert legacy controller stability.

Migration stages 1–5 are the current scope. Existing-controller adaptations and custom aircraft ports require the next individual review.
