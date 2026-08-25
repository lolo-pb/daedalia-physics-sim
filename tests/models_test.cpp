#include <cmath>
#include <cstdio>

#include "controllers/attitude_controller.hpp"
#include "controllers/controller_io.hpp"
#include "controllers/controller_selection.hpp"
#include "controllers/demo_controller.hpp"
#include "controllers/manual_controller.hpp"
#include "sensors/ideal_imu.hpp"

namespace {

bool Near(float actual, float expected, float tolerance = 1.0e-4f) {
    return std::abs(actual - expected) <= tolerance;
}

bool Check(bool condition, const char *message) {
    if (!condition) {
        std::fprintf(stderr, "FAILED: %s\n", message);
    }
    return condition;
}

} // namespace

int main() {
    bool passed = true;

    TargetDrone targets;
    targets.SetMotorTarget(MotorId::FrontLeft, -0.5f);
    targets.SetMotorTarget(MotorId::FrontRight, 1.5f);
    passed &= Check(Near(targets.GetMotorTarget(MotorId::FrontLeft), 0.0f), "negative motor target is clamped");
    passed &= Check(Near(targets.GetMotorTarget(MotorId::FrontRight), 1.0f), "motor target above one is clamped");

    TargetDrone demo_targets;
    UpdateDemoController(ControllerInput{}, demo_targets);
    passed &= Check(Near(demo_targets.GetMotorTarget(MotorId::FrontLeft), 0.8f), "front-left demo target is 0.8");
    passed &= Check(Near(demo_targets.GetMotorTarget(MotorId::FrontRight), 0.8f), "front-right demo target is 0.8");
    passed &= Check(Near(demo_targets.GetMotorTarget(MotorId::RearRight), 0.8f), "rear-right demo target is 0.8");
    passed &= Check(Near(demo_targets.GetMotorTarget(MotorId::RearLeft), 0.8f), "rear-left demo target is 0.8");

    FlightController selected_controller = FlightController::Demo;
    passed &= Check(
        SelectControllerSlot(2, selected_controller)
            && selected_controller == FlightController::ManualHover,
        "controller slot 2 selects manual hover");
    for (int slot = 3; slot <= 9; ++slot) {
        passed &= Check(
            !SelectControllerSlot(slot, selected_controller)
                && selected_controller == FlightController::ManualHover,
            "an unassigned controller slot preserves the active controller");
    }
    passed &= Check(
        !SelectControllerSlot(0, selected_controller)
            && selected_controller == FlightController::ManualHover,
        "unassigned controller slot 0 preserves the active controller");
    passed &= Check(
        SelectControllerSlot(1, selected_controller)
            && selected_controller == FlightController::Demo,
        "controller slot 1 selects demo");

    AttitudeController attitude_controller;
    ControllerInput attitude_input{};
    attitude_input.timestep_seconds = 0.01f;
    attitude_input.imu.body_specific_force_meters_per_second_squared.y = 9.81f;
    TargetDrone attitude_targets;
    attitude_controller.Update(
        attitude_input,
        AttitudeSetpoint{.throttle = 0.5f},
        attitude_targets);
    passed &= Check(
        Near(attitude_targets.GetMotorTarget(MotorId::FrontLeft), 0.5f)
            && Near(attitude_targets.GetMotorTarget(MotorId::FrontRight), 0.5f)
            && Near(attitude_targets.GetMotorTarget(MotorId::RearRight), 0.5f)
            && Near(attitude_targets.GetMotorTarget(MotorId::RearLeft), 0.5f),
        "level attitude produces equal motor targets");

    attitude_controller.Reset();
    attitude_controller.Update(
        attitude_input,
        AttitudeSetpoint{.pitch_rad = 0.1f, .throttle = 0.5f},
        attitude_targets);
    passed &= Check(
        attitude_targets.GetMotorTarget(MotorId::FrontLeft)
                > attitude_targets.GetMotorTarget(MotorId::RearLeft)
            && attitude_targets.GetMotorTarget(MotorId::FrontRight)
                > attitude_targets.GetMotorTarget(MotorId::RearRight),
        "positive pitch command raises front motor targets");

    ManualController manual_controller;
    ControllerInput manual_input{};
    manual_input.timestep_seconds = 0.01f;
    manual_input.imu.body_specific_force_meters_per_second_squared.y = 9.81f;
    TargetDrone manual_targets;
    manual_controller.Reset();
    manual_controller.Update(manual_input, manual_targets);
    passed &= Check(
        Near(manual_targets.GetMotorTarget(MotorId::FrontLeft), 0.70f)
            && Near(manual_targets.GetMotorTarget(MotorId::FrontRight), 0.70f)
            && Near(manual_targets.GetMotorTarget(MotorId::RearRight), 0.70f)
            && Near(manual_targets.GetMotorTarget(MotorId::RearLeft), 0.70f),
        "manual hover starts level at hover throttle");

    manual_controller.Reset();
    manual_input.keys.w = true;
    manual_controller.Update(manual_input, manual_targets);
    passed &= Check(
        manual_targets.GetMotorTarget(MotorId::FrontLeft)
                < manual_targets.GetMotorTarget(MotorId::RearLeft)
            && manual_targets.GetMotorTarget(MotorId::FrontRight)
                < manual_targets.GetMotorTarget(MotorId::RearRight),
        "manual W command tilts forward");
    manual_input.keys.w = false;
    manual_controller.Update(manual_input, manual_targets);
    passed &= Check(
        Near(manual_targets.GetMotorTarget(MotorId::FrontLeft), 0.70f)
            && Near(manual_targets.GetMotorTarget(MotorId::FrontRight), 0.70f)
            && Near(manual_targets.GetMotorTarget(MotorId::RearRight), 0.70f)
            && Near(manual_targets.GetMotorTarget(MotorId::RearLeft), 0.70f),
        "manual pitch returns to level when W is released");

    manual_controller.Reset();
    manual_input.keys.s = true;
    manual_controller.Update(manual_input, manual_targets);
    passed &= Check(
        manual_targets.GetMotorTarget(MotorId::FrontLeft)
                > manual_targets.GetMotorTarget(MotorId::RearLeft)
            && manual_targets.GetMotorTarget(MotorId::FrontRight)
                > manual_targets.GetMotorTarget(MotorId::RearRight),
        "manual S command tilts backward");
    manual_input.keys.s = false;

    manual_controller.Reset();
    manual_input.keys.a = true;
    manual_controller.Update(manual_input, manual_targets);
    passed &= Check(
        manual_targets.GetMotorTarget(MotorId::FrontLeft)
                < manual_targets.GetMotorTarget(MotorId::FrontRight)
            && manual_targets.GetMotorTarget(MotorId::RearLeft)
                < manual_targets.GetMotorTarget(MotorId::RearRight),
        "manual A command tilts left");
    manual_input.keys.a = false;

    manual_controller.Reset();
    manual_input.keys.d = true;
    manual_controller.Update(manual_input, manual_targets);
    passed &= Check(
        manual_targets.GetMotorTarget(MotorId::FrontLeft)
                > manual_targets.GetMotorTarget(MotorId::FrontRight)
            && manual_targets.GetMotorTarget(MotorId::RearLeft)
                > manual_targets.GetMotorTarget(MotorId::RearRight),
        "manual D command tilts right");
    manual_input.keys.d = false;

    manual_controller.Reset();
    manual_input.keys.q = true;
    manual_controller.Update(manual_input, manual_targets);
    const float yaw_front_left = manual_targets.GetMotorTarget(MotorId::FrontLeft);
    manual_input.keys.q = false;
    manual_controller.Update(manual_input, manual_targets);
    passed &= Check(
        manual_targets.GetMotorTarget(MotorId::FrontLeft)
                > manual_targets.GetMotorTarget(MotorId::FrontRight)
            && Near(manual_targets.GetMotorTarget(MotorId::FrontLeft), yaw_front_left),
        "manual yaw target is retained after Q is released");

    manual_controller.Reset();
    manual_input.timestep_seconds = 10.0f;
    manual_input.keys.r = true;
    manual_controller.Update(manual_input, manual_targets);
    manual_input.keys.r = false;
    manual_controller.Update(manual_input, manual_targets);
    passed &= Check(Near(manual_controller.GetThrottle(), 1.0f), "manual throttle persists and clamps at one");
    manual_input.keys.f = true;
    manual_controller.Update(manual_input, manual_targets);
    passed &= Check(Near(manual_controller.GetThrottle(), 0.0f), "manual throttle clamps at zero");
    manual_input.keys.f = false;
    manual_input.timestep_seconds = 0.01f;

    constexpr float timestep = 0.01f;
    const JPH::Vec3 gravity(0.0f, -9.81f, 0.0f);
    IdealImuModel imu;
    imu.Reset(JPH::Vec3::sZero());

    const ImuSample supported = imu.Sample(
        0.0, timestep, JPH::Quat::sIdentity(), JPH::Vec3::sZero(), JPH::Vec3::sZero(), gravity);
    passed &= Check(Near(supported.body_gyro_rad_per_second.y, 0.0f), "level stationary gyro is zero");
    passed &= Check(
        Near(supported.body_specific_force_meters_per_second_squared.y, 9.81f),
        "supported body-up specific force is +9.81 m/s^2");

    const ImuSample free_fall = imu.Sample(
        timestep,
        timestep,
        JPH::Quat::sIdentity(),
        JPH::Vec3::sZero(),
        gravity * timestep,
        gravity);
    passed &= Check(
        Near(free_fall.body_specific_force_meters_per_second_squared.y, 0.0f),
        "free-fall specific force is zero");

    imu.Reset(JPH::Vec3(5.0f, 0.0f, 0.0f));
    const ImuSample after_reset = imu.Sample(
        0.0,
        timestep,
        JPH::Quat::sIdentity(),
        JPH::Vec3::sZero(),
        JPH::Vec3(5.0f, 0.0f, 0.0f),
        gravity);
    passed &= Check(
        Near(after_reset.body_specific_force_meters_per_second_squared.x, 0.0f),
        "reset removes the velocity-history acceleration spike");

    return passed ? 0 : 1;
}
