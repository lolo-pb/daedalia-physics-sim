# Daedalia Physics Sim

The application is a native C++ simulator. SDL owns the window and OpenGL context, Jolt advances rigid-body physics, and the renderer draws the current Jolt body transforms.

Build and run:

```sh
cmake -S . -B build
cmake --build build
./build/daedalia
```

The app has a free-fly camera and a small ImGui physics panel for inspecting the box and controlling the simulation.

## Future work

- Drone-building API: simple rigid bodies with named motor and force points.
- Physics visualisation, saved scenarios, and experimental force models.
