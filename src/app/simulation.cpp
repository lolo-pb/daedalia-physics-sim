#include "simulation.hpp"

#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <thread>

#include <Jolt/Core/Factory.h>
#include <Jolt/Core/JobSystemThreadPool.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Body/BodyInterface.h>
#include <Jolt/Physics/Body/BodyLock.h>
#include <Jolt/Physics/Collision/BroadPhase/BroadPhaseLayer.h>
#include <Jolt/Physics/Collision/ObjectLayer.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/RegisterTypes.h>

#include "controllers/angle_mode_controller.hpp"
#include "controllers/demo_controller.hpp"
#include "controllers/horizon_mode_controller.hpp"
#include "controllers/position_hold_controller.hpp"
#include "drones/drone.hpp"
#include "drones/drone_definition.hpp"
#include "sensors/ideal_barometer.hpp"
#include "sensors/ideal_gps.hpp"
#include "sensors/ideal_imu.hpp"
#include "sensors/ideal_magnetometer.hpp"

namespace {

namespace Layers {
constexpr JPH::ObjectLayer Static = 0;
constexpr JPH::ObjectLayer Moving = 1;
constexpr JPH::uint NumLayers = 2;
}

namespace BroadPhaseLayers {
constexpr JPH::BroadPhaseLayer Static(0);
constexpr JPH::BroadPhaseLayer Moving(1);
constexpr JPH::uint NumLayers = 2;
}

class ObjectLayerPairFilter final : public JPH::ObjectLayerPairFilter {
public:
    bool ShouldCollide(JPH::ObjectLayer first, JPH::ObjectLayer second) const override {
        return first == Layers::Moving || second == Layers::Moving;
    }
};

class BroadPhaseLayerInterface final : public JPH::BroadPhaseLayerInterface {
public:
    JPH::uint GetNumBroadPhaseLayers() const override { return BroadPhaseLayers::NumLayers; }

    JPH::BroadPhaseLayer GetBroadPhaseLayer(JPH::ObjectLayer layer) const override {
        return layer == Layers::Static ? BroadPhaseLayers::Static : BroadPhaseLayers::Moving;
    }

#if defined(JPH_EXTERNAL_PROFILE) || defined(JPH_PROFILE_ENABLED)
    const char *GetBroadPhaseLayerName(JPH::BroadPhaseLayer layer) const override {
        return layer == BroadPhaseLayers::Static ? "Static" : "Moving";
    }
#endif
};

class ObjectVsBroadPhaseLayerFilter final : public JPH::ObjectVsBroadPhaseLayerFilter {
public:
    bool ShouldCollide(JPH::ObjectLayer layer, JPH::BroadPhaseLayer broad_phase_layer) const override {
        return layer == Layers::Static ? broad_phase_layer == BroadPhaseLayers::Moving : true;
    }
};

void Trace(const char *format, ...) {
    va_list arguments;
    va_start(arguments, format);
    std::vfprintf(stderr, format, arguments);
    va_end(arguments);
}

#ifdef JPH_ENABLE_ASSERTS
bool AssertFailed(const char *expression, const char *message, const char *file, JPH::uint line) {
    std::fprintf(stderr, "%s:%u: %s (%s)\n", file, line, expression, message == nullptr ? "" : message);
    return true;
}
#endif

template <typename Vector>
bool IsFiniteVector(const Vector &vector) {
    return std::isfinite(vector.GetX())
        && std::isfinite(vector.GetY())
        && std::isfinite(vector.GetZ());
}

bool IsFiniteQuaternion(const JPH::Quat &rotation) {
    return IsFiniteVector(rotation) && std::isfinite(rotation.GetW());
}

bool IsFiniteSensorVector(const SensorVector3 &vector) {
    return std::isfinite(vector.x)
        && std::isfinite(vector.y)
        && std::isfinite(vector.z);
}

constexpr int WorkerThreadCount(unsigned hardware_thread_count) {
    if (hardware_thread_count <= 1) {
        return 1;
    }
    const unsigned available_worker_threads = hardware_thread_count - 1;
    const unsigned maximum_thread_count = static_cast<unsigned>(std::numeric_limits<int>::max());
    return static_cast<int>(std::min(available_worker_threads, maximum_thread_count));
}

static_assert(WorkerThreadCount(0) == 1);
static_assert(WorkerThreadCount(1) == 1);
static_assert(WorkerThreadCount(2) == 1);
static_assert(WorkerThreadCount(8) == 7);
static_assert(WorkerThreadCount(std::numeric_limits<unsigned>::max()) == std::numeric_limits<int>::max());

bool HasSafeMotorCommandBehavior() {
    MotorCommands commands(2);
    commands.SetMotor(0, -0.5f);
    commands.SetMotor(1, 1.5f);
    commands.SetMotor(2, 0.5f);
    const bool values_are_bounded = commands.GetMotor(0) == 0.0f
        && commands.GetMotor(1) == 1.0f
        && commands.GetMotor(2) == 0.0f;

    commands.SetMotor(0, std::numeric_limits<float>::quiet_NaN());
    commands.SetMotor(1, std::numeric_limits<float>::infinity());
    const bool nonfinite_values_are_zero = commands.GetMotor(0) == 0.0f
        && commands.GetMotor(1) == 0.0f;

    commands.SetMotor(0, 0.5f);
    commands.Clear();
    const bool clear_resets_commands = commands.GetMotor(0) == 0.0f;
    return values_are_bounded
        && nonfinite_values_are_zero
        && clear_resets_commands;
}

struct PhysicsWorld {
    JPH::TempAllocatorImpl temp_allocator;
    JPH::JobSystemThreadPool job_system;
    ObjectLayerPairFilter object_layer_pair_filter;
    BroadPhaseLayerInterface broad_phase_layer_interface;
    ObjectVsBroadPhaseLayerFilter object_vs_broad_phase_layer_filter;
    JPH::PhysicsSystem physics;

    PhysicsWorld() :
        temp_allocator(10 * 1024 * 1024),
        job_system(
            JPH::cMaxPhysicsJobs,
            JPH::cMaxPhysicsBarriers,
            WorkerThreadCount(std::thread::hardware_concurrency())) {
        physics.Init(
            1024,
            0,
            1024,
            1024,
            broad_phase_layer_interface,
            object_vs_broad_phase_layer_filter,
            object_layer_pair_filter);
        physics.SetGravity(JPH::Vec3(0.0f, -9.81f, 0.0f));

        JPH::BodyInterface &bodies = physics.GetBodyInterface();
        const JPH::BodyCreationSettings floor_settings(
            new JPH::BoxShape(JPH::Vec3(10.0f, 0.5f, 10.0f)),
            JPH::RVec3(0.0, -0.5, 0.0),
            JPH::Quat::sIdentity(),
            JPH::EMotionType::Static,
            Layers::Static);
        bodies.CreateAndAddBody(floor_settings, JPH::EActivation::DontActivate);
    }
};

} // namespace

struct Simulation::Impl {
    PhysicsWorld world;
    Drone drone;
    IdealImuModel imu_model;
    IdealGpsModel gps_model;
    IdealBarometerModel barometer_model;
    IdealMagnetometerModel magnetometer_model;
    MotorCommands motor_commands;
    DemoController demo_controller;
    AngleModeController angle_mode_controller;
    HorizonModeController horizon_mode_controller;
    PositionHoldController position_hold_controller;
    FlightController active_controller = FlightController::Demo;
    std::array<float, 3> gravity{0.0f, -9.81f, 0.0f};
    int physics_frequency_hz = 30;
    double physics_step = 1.0 / static_cast<double>(physics_frequency_hz);
    double simulation_time = 0.0;
    ImuSample latest_imu_sample;
    GpsSample latest_gps_sample;
    BarometerSample latest_barometer_sample;
    MagnetometerSample latest_magnetometer_sample;

    Impl() :
        drone(world.physics.GetBodyInterface(), CreateQuadcopterDefinition()),
        motor_commands(drone.GetMotorCount()) {
        imu_model.Reset(Bodies().GetLinearVelocity(DroneId()));
        SampleSensors();
    }

    JPH::BodyInterface &Bodies() {
        return world.physics.GetBodyInterface();
    }

    JPH::BodyID DroneId() const {
        return drone.GetBodyID();
    }

    void SampleSensors() {
        JPH::BodyInterface &bodies = Bodies();
        latest_imu_sample = imu_model.Sample(
            simulation_time,
            static_cast<float>(physics_step),
            bodies.GetRotation(DroneId()),
            bodies.GetAngularVelocity(DroneId()),
            bodies.GetLinearVelocity(DroneId()),
            world.physics.GetGravity());
        latest_gps_sample = gps_model.Sample(
            simulation_time,
            bodies.GetPosition(DroneId()),
            bodies.GetLinearVelocity(DroneId()));
        latest_barometer_sample = barometer_model.Sample(
            simulation_time,
            static_cast<float>(bodies.GetPosition(DroneId()).GetY()));
        latest_magnetometer_sample = magnetometer_model.Sample(
            simulation_time,
            bodies.GetRotation(DroneId()));
    }

    bool HasValidInitialSensorState() const {
        return std::fabs(latest_gps_sample.world_position_meters.x) < 1.0e-6f
            && std::fabs(latest_gps_sample.world_position_meters.y - 1.0f) < 1.0e-6f
            && std::fabs(latest_gps_sample.world_position_meters.z) < 1.0e-6f
            && std::fabs(latest_gps_sample.world_velocity_meters_per_second.x) < 1.0e-6f
            && std::fabs(latest_gps_sample.world_velocity_meters_per_second.y) < 1.0e-6f
            && std::fabs(latest_gps_sample.world_velocity_meters_per_second.z) < 1.0e-6f
            && std::fabs(BarometricAltitudeMeters(latest_barometer_sample.pressure_pascals) - 1.0f) < 1.0e-3f
            && std::fabs(latest_magnetometer_sample.body_magnetic_field_microteslas.x) < 1.0e-6f
            && std::fabs(latest_magnetometer_sample.body_magnetic_field_microteslas.y) < 1.0e-6f
            && std::fabs(latest_magnetometer_sample.body_magnetic_field_microteslas.z + 50.0f) < 1.0e-6f;
    }

    bool HasFiniteSimulationState() {
        JPH::BodyInterface &bodies = Bodies();
        return bodies.IsAdded(DroneId())
            && IsFiniteVector(bodies.GetPosition(DroneId()))
            && IsFiniteQuaternion(bodies.GetRotation(DroneId()))
            && IsFiniteVector(bodies.GetLinearVelocity(DroneId()))
            && IsFiniteVector(bodies.GetAngularVelocity(DroneId()))
            && IsFiniteSensorVector(latest_imu_sample.body_gyro_rad_per_second)
            && IsFiniteSensorVector(latest_imu_sample.body_specific_force_meters_per_second_squared)
            && IsFiniteSensorVector(latest_gps_sample.world_position_meters)
            && IsFiniteSensorVector(latest_gps_sample.world_velocity_meters_per_second)
            && std::isfinite(latest_barometer_sample.pressure_pascals)
            && latest_barometer_sample.pressure_pascals > 0.0f
            && IsFiniteSensorVector(latest_magnetometer_sample.body_magnetic_field_microteslas);
    }

    bool HasConsistentSensorState() {
        JPH::BodyInterface &bodies = Bodies();
        const GpsSample checked_gps_sample = gps_model.Sample(
            simulation_time,
            bodies.GetPosition(DroneId()),
            bodies.GetLinearVelocity(DroneId()));
        const JPH::RVec3 checked_position = bodies.GetPosition(DroneId());
        const JPH::Vec3 checked_velocity = bodies.GetLinearVelocity(DroneId());
        const SensorVector3 &magnetic_field = latest_magnetometer_sample.body_magnetic_field_microteslas;

        return std::fabs(
            BarometricAltitudeMeters(latest_barometer_sample.pressure_pascals)
            - latest_gps_sample.world_position_meters.y) < 1.0e-3f
            && std::fabs(
                magnetic_field.x * magnetic_field.x
                + magnetic_field.y * magnetic_field.y
                + magnetic_field.z * magnetic_field.z
                - 2500.0f) < 1.0e-2f
            && std::fabs(checked_gps_sample.world_position_meters.x - checked_position.GetX()) < 1.0e-6f
            && std::fabs(checked_gps_sample.world_position_meters.y - checked_position.GetY()) < 1.0e-6f
            && std::fabs(checked_gps_sample.world_position_meters.z - checked_position.GetZ()) < 1.0e-6f
            && std::fabs(checked_gps_sample.world_velocity_meters_per_second.x - checked_velocity.GetX()) < 1.0e-6f
            && std::fabs(checked_gps_sample.world_velocity_meters_per_second.y - checked_velocity.GetY()) < 1.0e-6f
            && std::fabs(checked_gps_sample.world_velocity_meters_per_second.z - checked_velocity.GetZ()) < 1.0e-6f;
    }

    bool HasDemoMotorTarget(float target) const {
        return motor_commands.GetMotor(0) == target
            && motor_commands.GetMotor(1) == target
            && motor_commands.GetMotor(2) == target
            && motor_commands.GetMotor(3) == target;
    }
};

PhysicsRuntime::PhysicsRuntime() {
    JPH::Trace = Trace;
#ifdef JPH_ENABLE_ASSERTS
    JPH::AssertFailed = AssertFailed;
#endif
    JPH::RegisterDefaultAllocator();
    JPH::Factory::sInstance = new JPH::Factory();
    JPH::RegisterTypes();
}

PhysicsRuntime::~PhysicsRuntime() {
    JPH::UnregisterTypes();
    delete JPH::Factory::sInstance;
    JPH::Factory::sInstance = nullptr;
}

Simulation::Simulation() : impl_(std::make_unique<Impl>()) {}

Simulation::~Simulation() = default;

void Simulation::SelectController(int slot) {
    const FlightController previous_controller = impl_->active_controller;
    if (!SelectControllerSlot(slot, impl_->active_controller)
        || impl_->active_controller == previous_controller) {
        return;
    }
    if (impl_->active_controller == FlightController::AngleMode) {
        impl_->angle_mode_controller.Reset();
    } else if (impl_->active_controller == FlightController::HorizonMode) {
        impl_->horizon_mode_controller.Reset();
    } else if (impl_->active_controller == FlightController::PositionHold) {
        impl_->position_hold_controller.Reset();
    }
}

void Simulation::Step(const ControllerKeys &controller_keys) {
    impl_->SampleSensors();
    const ControllerInput controller_input{
        impl_->latest_imu_sample,
        impl_->latest_gps_sample,
        impl_->latest_barometer_sample,
        impl_->latest_magnetometer_sample,
        static_cast<float>(impl_->physics_step),
        controller_keys,
    };
    impl_->motor_commands.Clear();
    if (impl_->active_controller == FlightController::Demo) {
        impl_->demo_controller.Update(controller_input, impl_->motor_commands);
    } else if (impl_->active_controller == FlightController::AngleMode) {
        impl_->angle_mode_controller.Update(
            controller_input, impl_->motor_commands);
    } else if (impl_->active_controller == FlightController::HorizonMode) {
        impl_->horizon_mode_controller.Update(
            controller_input, impl_->motor_commands);
    } else if (impl_->active_controller == FlightController::PositionHold) {
        impl_->position_hold_controller.Update(
            controller_input, impl_->motor_commands);
    }
    impl_->drone.SetMotorTargets(impl_->motor_commands);
    impl_->drone.UpdateMotors();
    impl_->drone.ApplyForces(impl_->Bodies());
    impl_->world.physics.Update(
        static_cast<float>(impl_->physics_step),
        1,
        &impl_->world.temp_allocator,
        &impl_->world.job_system);
    impl_->simulation_time += impl_->physics_step;
}

void Simulation::Reset() {
    impl_->drone.Reset(impl_->Bodies());
    impl_->imu_model.Reset(impl_->Bodies().GetLinearVelocity(impl_->DroneId()));
    impl_->simulation_time = 0.0;
    impl_->SampleSensors();
    impl_->angle_mode_controller.Reset();
    impl_->horizon_mode_controller.Reset();
    impl_->position_hold_controller.Reset();
}

int Simulation::RunSmokeTest() {
    const bool motor_commands_are_safe = HasSafeMotorCommandBehavior();
    const bool initial_sensor_state_is_valid = impl_->HasValidInitialSensorState();
    Step(ControllerKeys{});
    const bool demo_started_disarmed = impl_->HasDemoMotorTarget(0.0f);
    ControllerKeys arm_demo;
    arm_demo.x = true;
    Step(arm_demo);
    const bool demo_armed = impl_->HasDemoMotorTarget(0.8f);
    Step(arm_demo);
    const bool held_x_did_not_retrigger = impl_->HasDemoMotorTarget(0.8f);
    Step(ControllerKeys{});
    Step(arm_demo);
    const bool demo_disarmed = impl_->HasDemoMotorTarget(0.0f);
    Step(ControllerKeys{});
    Step(arm_demo);
    for (int step = 7; step < impl_->physics_frequency_hz; ++step) {
        Step(ControllerKeys{});
    }
    const bool demo_remained_armed = impl_->HasDemoMotorTarget(0.8f);

    const bool demo_state_is_valid = motor_commands_are_safe
        && initial_sensor_state_is_valid
        && demo_started_disarmed
        && demo_armed
        && held_x_did_not_retrigger
        && demo_disarmed
        && demo_remained_armed
        && impl_->HasFiniteSimulationState()
        && impl_->HasConsistentSensorState();
    if (!demo_state_is_valid) {
        std::fprintf(stderr, "Simulation smoke test failed\n");
        return EXIT_FAILURE;
    }

    Reset();
    SelectController(2);
    const bool angle_mode_selected =
        GetActiveController() == FlightController::AngleMode;
    SelectController(3);
    const bool horizon_mode_selected =
        GetActiveController() == FlightController::HorizonMode;
    SelectController(4);
    const bool position_hold_selected =
        GetActiveController() == FlightController::PositionHold;
    SelectController(9);
    const bool invalid_slot_was_ignored =
        GetActiveController() == FlightController::PositionHold;
    for (int step = 0; step < impl_->physics_frequency_hz * 5; ++step) {
        Step(ControllerKeys{});
    }
    const DroneInspection stationary_hold = InspectDrone();
    const bool stationary_hold_is_stable =
        std::fabs(stationary_hold.position.GetX()) < 0.1
        && std::fabs(stationary_hold.position.GetY() - 1.0) < 0.15
        && std::fabs(stationary_hold.position.GetZ()) < 0.1
        && stationary_hold.linear_velocity.Length() < 0.2f;

    ControllerKeys move_target;
    move_target.w = true;
    move_target.r = true;
    for (int step = 0; step < impl_->physics_frequency_hz * 2; ++step) {
        Step(move_target);
    }
    for (int step = 0; step < impl_->physics_frequency_hz * 8; ++step) {
        Step(ControllerKeys{});
    }
    const DroneInspection moved_hold = InspectDrone();
    const bool moved_target_is_held =
        std::fabs(moved_hold.position.GetX()) < 0.35
        && std::fabs(moved_hold.position.GetY() - 2.0) < 0.35
        && std::fabs(moved_hold.position.GetZ() + 2.0) < 0.35
        && moved_hold.linear_velocity.Length() < 0.5f;

    Reset();
    for (int step = 0; step < impl_->physics_frequency_hz * 3; ++step) {
        Step(ControllerKeys{});
    }
    const DroneInspection reset_hold = InspectDrone();
    const bool reset_target_was_captured =
        std::fabs(reset_hold.position.GetX()) < 0.1
        && std::fabs(reset_hold.position.GetY() - 1.0) < 0.15
        && std::fabs(reset_hold.position.GetZ()) < 0.1
        && reset_hold.linear_velocity.Length() < 0.2f;

    const bool state_is_valid = angle_mode_selected
        && horizon_mode_selected
        && position_hold_selected
        && invalid_slot_was_ignored
        && stationary_hold_is_stable
        && moved_target_is_held
        && reset_target_was_captured
        && impl_->HasFiniteSimulationState()
        && impl_->HasConsistentSensorState();
    if (!state_is_valid) {
        std::fprintf(
            stderr,
            "Position hold smoke test failed: stationary=(%.3f, %.3f, %.3f), "
            "moved=(%.3f, %.3f, %.3f), velocity=%.3f, "
            "reset=(%.3f, %.3f, %.3f)\n",
            stationary_hold.position.GetX(),
            stationary_hold.position.GetY(),
            stationary_hold.position.GetZ(),
            moved_hold.position.GetX(),
            moved_hold.position.GetY(),
            moved_hold.position.GetZ(),
            moved_hold.linear_velocity.Length(),
            reset_hold.position.GetX(),
            reset_hold.position.GetY(),
            reset_hold.position.GetZ());
        return EXIT_FAILURE;
    }
    std::printf("Simulation smoke test passed\n");
    return EXIT_SUCCESS;
}

FlightController Simulation::GetActiveController() const {
    return impl_->active_controller;
}

float Simulation::GetActiveControllerThrottle() const {
    if (impl_->active_controller == FlightController::HorizonMode) {
        return impl_->horizon_mode_controller.GetThrottle();
    }
    return impl_->angle_mode_controller.GetThrottle();
}

int Simulation::GetPhysicsFrequencyHz() const {
    return impl_->physics_frequency_hz;
}

double Simulation::GetPhysicsStepSeconds() const {
    return impl_->physics_step;
}

void Simulation::SetPhysicsFrequencyHz(int frequency_hz) {
    impl_->physics_frequency_hz = frequency_hz;
    impl_->physics_step = 1.0 / static_cast<double>(frequency_hz);
}

std::array<float, 3> Simulation::GetGravity() const {
    return impl_->gravity;
}

void Simulation::SetGravity(const std::array<float, 3> &gravity) {
    impl_->gravity = gravity;
    impl_->world.physics.SetGravity(JPH::Vec3(gravity[0], gravity[1], gravity[2]));
    impl_->Bodies().ActivateBody(impl_->DroneId());
}

JPH::RVec3 Simulation::GetDronePosition() const {
    return impl_->world.physics.GetBodyInterface().GetPosition(impl_->DroneId());
}

DroneInspection Simulation::InspectDrone() const {
    const JPH::BodyInterface &bodies = impl_->world.physics.GetBodyInterface();
    DroneInspection inspection{
        bodies.GetPosition(impl_->DroneId()),
        bodies.GetRotation(impl_->DroneId()),
        bodies.GetLinearVelocity(impl_->DroneId()),
        bodies.GetAngularVelocity(impl_->DroneId()),
    };
    JPH::BodyLockRead lock(impl_->world.physics.GetBodyLockInterface(), impl_->DroneId());
    if (lock.Succeeded()) {
        inspection.mass = 1.0f / lock.GetBody().GetMotionProperties()->GetInverseMass();
    }
    return inspection;
}

DroneRenderState Simulation::GetDroneRenderState() const {
    const JPH::BodyInterface &bodies = impl_->world.physics.GetBodyInterface();
    const JPH::RVec3 position = bodies.GetPosition(impl_->DroneId());
    const JPH::Quat rotation = bodies.GetRotation(impl_->DroneId());
    return {
        position,
        rotation,
        impl_->drone.GetBodyHalfExtent(),
        impl_->drone.GetMotorWorldPositions(position, rotation),
    };
}

const ImuSample &Simulation::GetLatestImuSample() const {
    return impl_->latest_imu_sample;
}

const GpsSample &Simulation::GetLatestGpsSample() const {
    return impl_->latest_gps_sample;
}

const BarometerSample &Simulation::GetLatestBarometerSample() const {
    return impl_->latest_barometer_sample;
}

const MagnetometerSample &Simulation::GetLatestMagnetometerSample() const {
    return impl_->latest_magnetometer_sample;
}
