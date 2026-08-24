# Daedalia Physics Sim

The application is a native C++ simulator. SDL owns the window and OpenGL context, Jolt advances rigid-body physics, and the renderer draws the current Jolt body transforms.

Build and run:

```sh
cmake -S . -B build
cmake --build build
./build/daedalia
```

The app has a free-fly camera and a small ImGui physics panel for inspecting the drone and controlling the simulation. The drone layout and motor model are in `src/drone.*`; controller code writes normalized motor commands in `src/controller.*`.

## Drone and controller API

`Drone` is one Jolt rigid body with four private motors. Each motor has a local attachment position, a local thrust direction, and a local reaction-torque direction. Directions are transformed with the drone body each physics step.

Controllers only write a motor command in the range `0.0` to `1.0`:

```cpp
drone.SetMotorCommand(MotorId::FrontLeft, 0.8f);
```

Motor order is `FrontLeft`, `FrontRight`, `RearRight`, `RearLeft`. Front is local `-Z`; left is local `-X`.

Commands outside the range are clamped. Controllers cannot alter motor placement, directions, speed limits, or thrust/torque coefficients. There is no motor lag yet: each physics step, speed is `command * max_speed_rad_per_second`. Thrust and reaction torque are calculated from speed squared and then applied to Jolt.

## Future work

- More drone layouts, physics visualisation, saved scenarios, and experimental force models.
