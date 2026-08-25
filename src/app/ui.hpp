#pragma once

class Simulation;

struct PhysicsPanelResult {
    bool reset = false;
    bool follow_drone_enabled = false;
    bool physics_frequency_changed = false;
};

PhysicsPanelResult DrawPhysicsPanel(
    Simulation &simulation,
    bool &paused,
    bool &single_step,
    bool &follow_drone);
void DrawSensorsPanel(const Simulation &simulation);
