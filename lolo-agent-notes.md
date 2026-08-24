# Daedalia Physics Sim

The application is a native C++ simulator. SDL owns the window and OpenGL context, Jolt advances rigid-body physics, and the renderer draws the current Jolt body transforms.

Build and run:

```sh
cmake -S . -B build
cmake --build build
./build/daedalia
```

The app has a free-fly camera and a small ImGui physics panel for inspecting the drone and controlling the simulation. The drone layout and motor model are in `src/drone.*`; controller code writes normalized motor commands in `src/controller.*`.

## Controller and simulation models

`src/controller_io.hpp` is the shared, physics-independent controller contract. A controller receives an `ImuSample`, fixed timestep, and a simulator-owned `TargetDrone` that exposes only four normalized motor targets. `TargetDrone::SetMotorTarget` clamps each target to `0.0` through `1.0`.

The fixed-step flow is:

```text
Jolt state -> ideal IMU -> controller -> motor targets -> motor model -> forces/torques -> Jolt update
```

`src/ideal_imu.*` is where sensor behavior is programmed. It currently converts Jolt truth into ideal body-frame gyro and specific force. It derives acceleration from velocity history, which is reset with the drone.

`Drone` in `src/drone.*` is one Jolt rigid body with four private motors. Each motor keeps its normalized target separate from its physical speed. `SetMotorTargets`, `UpdateMotors`, and `ApplyForces` are separate phases. Motor speed behavior is programmed in `UpdateMotors`; thrust and reaction torque behavior is programmed in `ApplyForces`. The current model has no lag and both forces remain proportional to speed squared.

Motor order is `FrontLeft`, `FrontRight`, `RearRight`, `RearLeft`. Local `+X` is right, `+Y` is up, and `-Z` is front.

Ground-truth position, attitude, and velocities remain available only to simulation and the debug UI. The UI also shows the latest controller-facing IMU sample.

## Future work

- Build and tune a proper PID flight controller for the quadcopter.
- Once the quadcopter is stable, try a weird multirotor layout or a helicopter.
- More drone layouts, physics visualisation, saved scenarios, and experimental force models.
