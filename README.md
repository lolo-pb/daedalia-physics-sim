# Daedalia Physics Sim

Daedalia is a native C++ test bed for programming simulated sensors and flight controllers.

The goal is to experiment with control software against a physical vehicle model: the simulator turns rigid-body state into sensor readings, passes those readings to a controller, and applies the controller's motor commands back to the vehicle. The current vehicle is a quadcopter, but the same approach is intended to support other aircraft, sensor models, and control strategies.

```text
Jolt physics -> simulated sensors -> flight controller -> motor model -> forces and torques
```

![Daedalia Physics Sim running](screenshots/image.png)

## Current state

- A Jolt rigid-body simulation running at a configurable fixed timestep.
- An ideal IMU that reports body-frame angular velocity and specific force.
- A controller interface that receives only the IMU sample and timestep.
- Four normalized motor outputs driving quadcopter thrust and reaction torque.
- An OpenGL view and ImGui panel for inspecting ground truth, sensor output, and simulation state.

The included demo controller currently applies constant throttle. It is a simple integration check, not a stable flight controller.

## Build and run

Requirements are a C++20 compiler, CMake 3.24 or newer, Python 3, and OpenGL development libraries.

```sh
cmake -S . -B build
cmake --build build
./build/daedalia
```

CMake downloads SDL, Jolt Physics, GLM, glad, and Dear ImGui during the first configuration.

To run the model tests:

```sh
ctest --test-dir build --output-on-failure
```

## Controls

- Hold the right mouse button and move the mouse to look around.
- Use `W`, `A`, `S`, and `D` to move.
- Use `Space` to rise and `Ctrl` to descend.
- Hold `Shift` to move faster.

The `Physics` panel can pause, reset, or single-step the simulation, adjust gravity and timestep, and compare the drone's ground-truth state with the IMU values available to the controller.

## Extending the simulator

- Sensor behavior lives in `src/sensors/`.
- Flight controllers and their physics-independent input/output contract live in `src/controllers/`.
- Vehicle geometry, motors, and force application live in `src/drone.*`.
- `src/main.cpp` owns the fixed-step simulation loop, rendering, and debug UI.

Near-term work is to replace the constant-throttle demo with a tuned closed-loop controller, then add realistic sensor imperfections and experiment with different aircraft layouts.
