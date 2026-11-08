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

## Headless altitude recording

```sh
./build/daedalia --altitude-test f450 position-hold 30
```

Arguments are aircraft, controller, and positive duration in simulated seconds.
F450 controllers: `demo`, `angle`, `horizon`, `position-hold`. C172: `direct`.
Runs without a window or keyboard input, as fast as possible, using normal startup conditions.
Controllers requiring manual arming or throttle may remain idle.

Each run creates a CSV such as `outputs/f450_position-hold_1.csv` and prints its path.
Numbering starts at 1 and continues above the highest existing CSV or PNG number for that aircraft/controller pair.
Only filenames are inspected; older timestamp-named files are left untouched.
The output directory is relative to the launch directory.
Columns are `time_seconds,height_change_ft`: true altitude change from the starting position, positive upward.
Samples include time zero and every 60 Hz tick through the last tick at or before the requested duration.
Durations shorter than one tick produce only the initial sample.
The flag cannot be combined with `--aircraft` or `--smoke-test`.

Add `--plot` to save a PNG graph beside the CSV:

```sh
./build/daedalia --altitude-test f450 position-hold 30 --plot
python3 outputs/plot_altitude.py outputs/your_recording.csv
```

The plotting helper lives in `outputs/plot_altitude.py`; generated CSVs and PNGs are ignored by Git.
It requires Matplotlib in the Python interpreter selected by CMake (`Python3_EXECUTABLE`).
C++ launches that interpreter after closing the CSV, waits for the script, and reports plotting failures while preserving the CSV.
The script uses a headless backend; no display is required. Keep the source plotting script available at its configured path.
