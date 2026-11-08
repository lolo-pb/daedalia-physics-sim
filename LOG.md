## 10/04/2026

I just made the triangular bypiramid (TBC) drone fly
it kind of holds altitude but tends to spiral out of 
controll .

i gelieve high angula velocities fucks with simulated
imu readings.

<important!> i noticed that it goes crazy if you switch to other controllers, if not its scarily stable, still fails long term

## 10/12/2026 — JSBSim migration

Replaced Jolt completely with JSBSim 1.3.2, including ground reactions. The bundled F450 quadcopter and C172 are selectable, with each aircraft's physics, graphics and controller choices grouped in one AircraftDefinition.

Kept programmable sensors and a unified ControllerInput / ControllerOutput API. Samples use native JSBSim units; conversions belong to controllers. Manual input is a normal controller. The configurable tick defaults to 60 Hz, with smaller internal physics steps where needed.

Connected the existing OpenGL pipeline to aircraft pose, propellers and moving control surfaces. Updated README and lolo-notes.md, including a Mermaid file-flow graph.

Build, both backend smoke checks, longer runs and offscreen rendering checks passed. The stock F450 still emits model-version and internal sensor-binding warnings; full model validation is separate work.

Stopped before adapting existing controller algorithms, which remain untouched and excluded from the build. Direct-motor wiring and custom drone ports await review and approval.
