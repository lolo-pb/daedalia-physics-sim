#include <cmath>
#include <cstdio>

#include "controllers/controller_io.hpp"
#include "controllers/demo_controller.hpp"
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
