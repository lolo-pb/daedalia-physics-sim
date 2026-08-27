---
name: write-daedalia-controllers
description: Add or modify flight controllers in the Daedalia physics simulator. Use when implementing controller logic, wiring a controller into the simulator, or changing controller inputs and motor outputs.
---

# Write Daedalia Controllers

Keep controller work scoped to `src/controllers/`. If integration requires a small change elsewhere ask and wait for confirmation.
Do not alter the drone or physics model unless the user asks for it. Do not modify other controllers, keep evrthing scoped to the one you are told to edit or create.

Also ask and wait for confirmation when you believe the controller requires new sensors, dont add sensors unless given permission.

## Read First

- For a new selectable controller, inspect `controller_selection.hpp`, `src/app/simulation.cpp`, and the controller target in `CMakeLists.txt`.

## Controller Contract

- Consume sensor samples, the fixed timestep, and control keys through `ControllerInput`.
- Command the aircraft only through `TargetDrone::SetMotorTarget`.
- Treat motor targets as normalized values from `0.0` to `1.0`.
- Keep simulator ground truth out of controller code unless the user explicitly requests a non-realistic controller.

## Workflow

1. State the intended behavior and any assumptions about controls, sensors, and tuning.
2. Add the smallest controller interface and implementation that fits the existing style.
3. Wire new controllers into the build, selection enum/slot mapping, simulation storage, dispatch, and reset path.
4. Update controller-facing UI labels or help text only when selection or controls change.

## Verification

Run:

```sh
cmake --build build
ctest --test-dir build --output-on-failure
```

For tuning-sensitive behavior, report what was verified automatically and what still needs interactive flight testing.

## Repository-Specific Notes

Extend this section with controller conventions, tuning rules, test scenarios, or known pitfalls as they are established.
