# Daedalia Physics Sim

## Structure

- `src/app/`: startup selection, window, fixed-step loop, debug UI, camera and OpenGL.
- `src/aircraft/`: one AircraftDefinition per aircraft, referencing its physics model, initial conditions, graphics parts, control bindings and available controllers.
- `src/physics/`: JSBSim backend and native aircraft state.
- `src/sensors/`: programmable ideal IMU, local GPS, barometer and magnetometer.
- `src/controllers/`: unified input/output API and ManualController. Original controller implementations remain unbuilt pending review.

F450 and C172 are bundled JSBSim models. Their representative geometry is made from simple parts; pose, propellers and surface movement follow actual simulation state.

## Flow

```text
main -> startup aircraft selector -> AircraftDefinition
     -> Simulation + JSBSim -> sensors -> selected controller
     -> ControllerOutput -> JSBSim step
     -> latest state -> UI and OpenGL aircraft parts
```

Aircraft is chosen at startup. Each aircraft definition supplies its controller choices; both currently offer Manual. Reset rebuilds the model at its on-ground starting state and clears manual commands. Default frequency is 60 Hz, configurable in the UI.

Each aircraft also owns its maximum JSBSim integration interval: F450 0.002 seconds, C172 1/120 second, matching bundled script defaults. Physics substeps hold controller commands; sensors and controllers run once per configured tick.

## Units and responsibilities

Physics and sensor samples keep JSBSim native units: feet, ft/s, ft/s², psf, slugs and radians. Body axes are forward/right/down; GPS is local NED. Controllers perform any conversions they need.

Rendering keeps lengths in feet and maps axes to X right, Y up, -Z forward. Ground contact is JSBSim only. No Jolt or arbitrary mesh collision.

Manual uses the bundled aircraft controls, including F450's stock rate stabilization. Raw indexed motor targets remain in the unified output API for later controller integration; they are not yet connected to JSBSim.

Ground truth is not exposed through ControllerInput. Each controller owns its sensor interpretation, fusion, units, PID state and mixing. Selectable controllers use `_controller` names; shared utilities such as `pid` stay independent.

Stock F450 data emits version and internal sensor-binding warnings. Both backend checks pass and a longer run showed lift-off, but model validation/tuning is still separate work.

## Build

```sh
cmake -S . -B build
cmake --build build -j4
./build/daedalia --aircraft f450
./build/daedalia --aircraft c172
ctest --test-dir build --output-on-failure
```

CMake fetches JSBSim 1.3.2 and the graphics dependencies. Model data is copied to build/jsbsim-data and resolved independently of the launch directory.

## Migration pause

Stop after stages 1–5. Review existing controllers and custom aircraft one by one with the user before adapting them. Old drone definitions are recoverable through Git history; no custom physics model has been ported yet.

## Later work

- Individual controller and custom aircraft migrations.
- Sensor noise and independent sample rates.
- Telemetry CSV and movement trails.
- More detailed aircraft assets only if wanted.
- Map geometry (visual only until terrain/ground support is deliberately extended).
- Preserved custom-aircraft ideas: verify triangular-bipyramid torque directions, add its altitude and GPS position hold, and explore a 4-by-4-rotor zero-gravity aircraft. These are deferred, not automatically supported by the bundled models.
