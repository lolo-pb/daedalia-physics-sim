# Daedalia Physics Sim

Minimal native physics and graphics bootstrap: one box falls onto a ground plane.

## Build and run

```sh
cmake -S . -B build
cmake --build build
./build/daedalia
```

CMake downloads the project dependencies on first configuration.

## Camera controls

- Hold right mouse button to look around.
- `W` `A` `S` `D` move.
- `Space` rises; `Ctrl` descends; `Shift` boosts movement speed.
