# Daedalia Physics Sim

The application is a native C++ simulator. SDL owns the window and OpenGL context, Jolt advances rigid-body physics, and the renderer draws the current Jolt body transforms.

Build and run:

```sh
cmake -S . -B build
cmake --build build
./build/daedalia
```

Run the headless simulation smoke test with:

```sh
ctest --test-dir build --output-on-failure
```

The smoke test uses the app's real fixed-step physics path but exits before SDL and window setup.

The app has a free-fly camera and a small ImGui physics panel for inspecting the drone, controlling the simulation, and switching flight controllers. The drone layout and motor model are in `src/drone.*`; controller code lives in `src/controllers/` and writes normalized motor commands.

## Controller and simulation models

`src/controllers/controller_io.hpp` is the shared, physics-independent controller contract. A controller receives an `ImuSample`, fixed timestep, raw control keys, and a simulator-owned `TargetDrone` that exposes only four normalized motor targets. `TargetDrone::SetMotorTarget` clamps each target to `0.0` through `1.0`.

The app can switch controllers at runtime with number keys or the ImGui panel. The demo controller applies constant throttle. Manual Hover interprets keyboard input as attitude, yaw, and throttle commands, then uses the attitude PID to produce motor targets.

The fixed-step flow is:

```text
Jolt state -> ideal IMU -> controller -> motor targets -> motor model -> forces/torques -> Jolt update
```

`src/sensors/ideal_imu.*` is where sensor behavior is programmed. It currently converts Jolt truth into ideal body-frame gyro and specific force. It derives acceleration from velocity history, which is reset with the drone.

`Drone` in `src/drone.*` is one Jolt rigid body with four private motors. Each motor keeps its normalized target separate from its physical speed. `SetMotorTargets`, `UpdateMotors`, and `ApplyForces` are separate phases. Motor speed behavior is programmed in `UpdateMotors`; thrust and reaction torque behavior is programmed in `ApplyForces`. The current model has no lag and both forces remain proportional to speed squared.

Motor order is `FrontLeft`, `FrontRight`, `RearRight`, `RearLeft`. Local `+X` is right, `+Y` is up, and `-Z` is front.

Ground-truth position, attitude, and velocities remain available only to simulation and the debug UI. The UI also shows the latest controller-facing IMU sample.

## Flow

Startup :

  main
   ├─ initialize Jolt runtime
   ├─ construct Simulation
   │   ├─ create physics world and floor
   │   ├─ create drone
   │   ├─ create sensors/controllers
   │   └─ take initial sensor samples
   └─ run interactive application
        ├─ initialize SDL, OpenGL, and ImGui
        ├─ create Renderer
        └─ repeat every frame
             1. process events
             2. draw/update UI controls
             3. read keyboard input
             4. advance fixed physics steps
             5. update camera follow
             6. render scene and ImGui
             7. swap window buffers


Working loop :

One call to Simulation::Step() performs:

     sample sensors
         ↓
     run selected controller
         ↓
     set motor targets
         ↓
     update motor speeds
         ↓
     apply forces and torques
         ↓
     advance Jolt physics
         ↓
     advance simulation time

     This is implemented at src/app/simulation.cpp:287.

## File responsibilities

   File                Responsibility
  ━━━━━━━━━━━━━━━━━━  ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
   src/main.cpp        Program mode selection and Jolt runtime
                       lifecycle
  ──────────────────  ────────────────────────────────────────────
   src/app/            Window, events, camera, timing, input, and
   application.cpp     frame loop
  ──────────────────  ────────────────────────────────────────────
   src/app/            Public interface between the app and
   simulation.hpp      simulation
  ──────────────────  ────────────────────────────────────────────
   src/app/            Jolt world, drone, sensors, controllers,
   simulation.cpp      stepping, reset, and smoke validation
  ──────────────────  ────────────────────────────────────────────
   src/app/ui.cpp      ImGui panels and UI-to-simulation actions
  ──────────────────  ────────────────────────────────────────────
   src/app/            Shaders, meshes, OpenGL drawing, and scene
   renderer.cpp        transforms


## Future work

- Tune and validate the attitude PID against the simulated quadcopter.
- Once the quadcopter is stable, try a weird multirotor layout or a helicopter.
- More drone layouts, physics visualisation, saved scenarios, and experimental force models.

- Make separate menus in ui for physics real stuff and sensor data
