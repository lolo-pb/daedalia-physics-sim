# Daedalia Physics Sim

Daedalia is a native C++ flight-physics test bed. SDL owns the window and OpenGL context, Jolt advances rigid-body physics, ImGui provides debug controls, and the renderer draws the latest simulated state.

Build and run:

```sh
cmake -S . -B build
cmake --build build
./build/daedalia
```

Run the headless smoke test:

```sh
ctest --test-dir build --output-on-failure
```

## Structure

- `src/app/` owns startup, the interactive loop, fixed-step simulation, UI, camera, and rendering.
- `src/sensors/` converts Jolt state into ideal IMU, GPS, barometer, and magnetometer samples.
- `src/controllers/` receives sensor samples and input keys, then writes normalized indexed motor commands.
- `src/drones/` defines aircraft layouts and runs their bodies and motors in Jolt.
- `src/main.cpp` selects interactive or headless mode and owns the Jolt runtime lifetime.

Ground-truth position, attitude, and velocity stay inside the simulation and debug UI. Controllers only receive sensor samples.

## Drone model

`DroneDefinition` is a blueprint. It contains the body box size, mass, starting pose, and a variable-length list of motors. Each motor definition provides its local position, thrust direction, reaction-torque direction, maximum speed, and force coefficients.

`CreateQuadcopterDefinition()` and `CreateTricopterDefinition()` supply the current aircraft layouts. The generic runtime `Drone` consumes the selected definition, creates the Jolt body, stores changing motor state, and applies motor forces and torques. Rendering also uses the definition's body size and the runtime motor-position list, so it does not assume a fixed motor count.

Current quad motor indices are:

```text
0 front-left
1 front-right
2 rear-right
3 rear-left
```

Local `+X` is right, `+Y` is up, and `-Z` is front.

Current tricopter motor indices are:

```text
0 front-left
1 front-right
2 rear
```

## Controller command boundary

`MotorCommands` is the physics-independent output API. Controllers call `SetMotor(index, target)` without receiving the selected drone type or motor count.

- The command buffer is sized from the physical drone and cleared before every controller update.
- Missing commands therefore remain at zero for that step.
- Extra indices are ignored.
- Finite targets are clamped to `0.0` through `1.0`.
- NaN and infinity become zero before reaching the drone.

This keeps controllers independent from aircraft definitions. A controller may be physically unsuitable for a layout, but mismatched motor commands remain safe.

## Controller organization

- Selectable flight modes that own user controls and motor mixing use `_controller` names.
- Reusable control utilities are named by their function without `_controller`, such as `pid` and `attitude`.
- `pid` owns generic PID state and limits. `attitude` estimates orientation and converts attitude setpoints into pitch, roll, and yaw corrections.
- Motor mixing stays in the airframe-specific flight controller because motor indices and geometry belong to the aircraft layout.

## Flow

Startup:

```text
main
  -> initialize Jolt runtime
  -> select an aircraft or use the command-line choice
  -> construct Simulation
       -> create physics world and floor
       -> create the selected drone definition
       -> create generic Drone from definition
       -> size motor command buffer from the drone
       -> create sensors and controllers
  -> initialize SDL, OpenGL, ImGui, and Renderer
  -> play the skippable startup animation
  -> run the interactive frame loop
```

One fixed simulation step:

```text
Jolt state
  -> sample sensors
  -> clear motor commands
  -> run selected controller
  -> copy commands for motors that physically exist
  -> update motor speeds
  -> apply thrust and reaction torque
  -> advance Jolt physics
  -> advance simulation time
```

Switching controllers resets the newly selected stateful controller but leaves the drone's physical state untouched. Its next output replaces the motor commands on the next fixed step.

## Future work

- <Important!> thers a rotational bug, where pitching while yawing offsets the target attitude.

- Tune and validate the controllers against the quadcopter.
- Add more layouts, saved scenarios, and experimental sensor or force models.
