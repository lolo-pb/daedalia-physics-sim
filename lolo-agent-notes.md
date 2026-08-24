# Daedalia Physics Sim

The application is a native C++ simulator. SDL owns the window and OpenGL context, Jolt advances rigid-body physics, and the renderer draws the current Jolt body transforms.

Build and run:

```sh
cmake -S . -B build
cmake --build build
./build/daedalia
```

The app has a free-fly camera and a small ImGui physics panel for inspecting the drone and controlling the simulation. The drone layout and demo controller are C++ code in `src/drone.*` and `src/controller.*`.

## Future work

- More drone layouts, physics visualisation, saved scenarios, and experimental force models.
