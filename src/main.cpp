#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <thread>

#include <glad/gl.h>
#include <SDL3/SDL.h>

#include <imgui.h>
#include <imgui_impl_opengl3.h>
#include <imgui_impl_sdl3.h>

#include <glm/ext/matrix_transform.hpp>
#include <glm/ext/quaternion_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <Jolt/Jolt.h>
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

#include "controllers/demo_controller.hpp"
#include "drone.hpp"
#include "sensors/ideal_imu.hpp"

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

GLuint CompileShader(GLenum type, const char *source) {
    const GLuint shader = glCreateShader(type);
    glShaderSource(shader, 1, &source, nullptr);
    glCompileShader(shader);

    GLint compiled = GL_FALSE;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &compiled);
    if (compiled == GL_FALSE) {
        char log[1024];
        glGetShaderInfoLog(shader, sizeof(log), nullptr, log);
        std::fprintf(stderr, "Shader compilation failed: %s\n", log);
        std::exit(EXIT_FAILURE);
    }
    return shader;
}

GLuint CreateProgram() {
    constexpr char vertex_source[] = R"(
        #version 450 core
        layout (location = 0) in vec3 position;
        layout (location = 1) in vec3 normal;
        uniform mat4 model;
        uniform mat4 view_projection;
        out vec3 world_normal;
        void main() {
            world_normal = mat3(model) * normal;
            gl_Position = view_projection * model * vec4(position, 1.0);
        }
    )";
    constexpr char fragment_source[] = R"(
        #version 450 core
        in vec3 world_normal;
        uniform vec3 colour;
        out vec4 fragment_colour;
        void main() {
            float light = max(dot(normalize(world_normal), normalize(vec3(0.4, 1.0, 0.3))), 0.0);
            fragment_colour = vec4(colour * (0.25 + 0.75 * light), 1.0);
        }
    )";

    const GLuint program = glCreateProgram();
    const GLuint vertex_shader = CompileShader(GL_VERTEX_SHADER, vertex_source);
    const GLuint fragment_shader = CompileShader(GL_FRAGMENT_SHADER, fragment_source);
    glAttachShader(program, vertex_shader);
    glAttachShader(program, fragment_shader);
    glLinkProgram(program);
    glDeleteShader(vertex_shader);
    glDeleteShader(fragment_shader);
    return program;
}

struct Mesh {
    GLuint vao = 0;
    GLuint vertex_count = 0;
};

struct Camera {
    glm::vec3 position{7.0f, 5.0f, 7.0f};
    float yaw = -135.0f;
    float pitch = -20.0f;

    glm::vec3 Forward() const {
        const float yaw_radians = glm::radians(yaw);
        const float pitch_radians = glm::radians(pitch);
        return glm::normalize(glm::vec3(
            std::cos(yaw_radians) * std::cos(pitch_radians),
            std::sin(pitch_radians),
            std::sin(yaw_radians) * std::cos(pitch_radians)));
    }
};

Mesh CreateMesh(const float *vertices, GLuint vertex_count) {
    Mesh mesh{.vertex_count = vertex_count};
    GLuint vertex_buffer = 0;
    glCreateVertexArrays(1, &mesh.vao);
    glCreateBuffers(1, &vertex_buffer);
    glNamedBufferData(vertex_buffer, static_cast<GLsizeiptr>(vertex_count * 6 * sizeof(float)), vertices, GL_STATIC_DRAW);
    glVertexArrayVertexBuffer(mesh.vao, 0, vertex_buffer, 0, 6 * sizeof(float));
    glEnableVertexArrayAttrib(mesh.vao, 0);
    glEnableVertexArrayAttrib(mesh.vao, 1);
    glVertexArrayAttribFormat(mesh.vao, 0, 3, GL_FLOAT, GL_FALSE, 0);
    glVertexArrayAttribFormat(mesh.vao, 1, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float));
    glVertexArrayAttribBinding(mesh.vao, 0, 0);
    glVertexArrayAttribBinding(mesh.vao, 1, 0);
    return mesh;
}

void DrawMesh(const Mesh &mesh, GLuint program, const glm::mat4 &model, const glm::vec3 &colour) {
    glUniformMatrix4fv(glGetUniformLocation(program, "model"), 1, GL_FALSE, glm::value_ptr(model));
    glUniform3fv(glGetUniformLocation(program, "colour"), 1, glm::value_ptr(colour));
    glBindVertexArray(mesh.vao);
    glDrawArrays(GL_TRIANGLES, 0, static_cast<GLsizei>(mesh.vertex_count));
}

} // namespace

int main() {
    JPH::Trace = Trace;
#ifdef JPH_ENABLE_ASSERTS
    JPH::AssertFailed = AssertFailed;
#endif
    JPH::RegisterDefaultAllocator();
    JPH::Factory::sInstance = new JPH::Factory();
    JPH::RegisterTypes();

    JPH::TempAllocatorImpl temp_allocator(10 * 1024 * 1024);
    JPH::JobSystemThreadPool job_system(
        JPH::cMaxPhysicsJobs,
        JPH::cMaxPhysicsBarriers,
        std::max(1u, std::thread::hardware_concurrency() - 1));
    ObjectLayerPairFilter object_layer_pair_filter;
    BroadPhaseLayerInterface broad_phase_layer_interface;
    ObjectVsBroadPhaseLayerFilter object_vs_broad_phase_layer_filter;
    JPH::PhysicsSystem physics;
    physics.Init(
        1024,
        0,
        1024,
        1024,
        broad_phase_layer_interface,
        object_vs_broad_phase_layer_filter,
        object_layer_pair_filter);
    float gravity[] = {0.0f, -9.81f, 0.0f};
    physics.SetGravity(JPH::Vec3(gravity[0], gravity[1], gravity[2]));
    double physics_step = 1.0 / 120.0;

    JPH::BodyInterface &bodies = physics.GetBodyInterface();
    const JPH::BodyCreationSettings floor_settings(
        new JPH::BoxShape(JPH::Vec3(10.0f, 0.5f, 10.0f)),
        JPH::RVec3(0.0, -0.5, 0.0),
        JPH::Quat::sIdentity(),
        JPH::EMotionType::Static,
        Layers::Static);
    bodies.CreateAndAddBody(floor_settings, JPH::EActivation::DontActivate);

    Drone drone(bodies);
    const JPH::BodyID drone_id = drone.GetBodyID();
    IdealImuModel imu_model;
    imu_model.Reset(bodies.GetLinearVelocity(drone_id));
    double simulation_time = 0.0;
    ImuSample latest_imu_sample = imu_model.Sample(
        simulation_time,
        static_cast<float>(physics_step),
        bodies.GetRotation(drone_id),
        bodies.GetAngularVelocity(drone_id),
        bodies.GetLinearVelocity(drone_id),
        physics.GetGravity());
    TargetDrone target_drone;

    if (!SDL_Init(SDL_INIT_VIDEO)) {
        std::fprintf(stderr, "SDL initialization failed: %s\n", SDL_GetError());
        return EXIT_FAILURE;
    }
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 4);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 5);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
    SDL_Window *window = SDL_CreateWindow("Daedalia Physics Sim", 1280, 720, SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE);
    if (window == nullptr) {
        std::fprintf(stderr, "Window creation failed: %s\n", SDL_GetError());
        return EXIT_FAILURE;
    }
    SDL_GLContext context = SDL_GL_CreateContext(window);
    if (context == nullptr || !gladLoadGL(reinterpret_cast<GLADloadfunc>(SDL_GL_GetProcAddress))) {
        std::fprintf(stderr, "OpenGL initialization failed: %s\n", SDL_GetError());
        return EXIT_FAILURE;
    }
    SDL_GL_SetSwapInterval(1);
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui::StyleColorsDark();
    ImGui_ImplSDL3_InitForOpenGL(window, context);
    ImGui_ImplOpenGL3_Init("#version 450");

    constexpr float cube_vertices[] = {
        -0.5f,-0.5f, 0.5f, 0, 0, 1,  0.5f,-0.5f, 0.5f, 0, 0, 1,  0.5f, 0.5f, 0.5f, 0, 0, 1,  -0.5f,-0.5f, 0.5f, 0, 0, 1,  0.5f, 0.5f, 0.5f, 0, 0, 1,  -0.5f, 0.5f, 0.5f, 0, 0, 1,
        -0.5f,-0.5f,-0.5f, 0, 0,-1, -0.5f, 0.5f,-0.5f, 0, 0,-1,  0.5f, 0.5f,-0.5f, 0, 0,-1, -0.5f,-0.5f,-0.5f, 0, 0,-1,  0.5f, 0.5f,-0.5f, 0, 0,-1,  0.5f,-0.5f,-0.5f, 0, 0,-1,
        -0.5f, 0.5f,-0.5f, 0, 1, 0, -0.5f, 0.5f, 0.5f, 0, 1, 0,  0.5f, 0.5f, 0.5f, 0, 1, 0, -0.5f, 0.5f,-0.5f, 0, 1, 0,  0.5f, 0.5f, 0.5f, 0, 1, 0,  0.5f, 0.5f,-0.5f, 0, 1, 0,
        -0.5f,-0.5f,-0.5f, 0,-1, 0,  0.5f,-0.5f,-0.5f, 0,-1, 0,  0.5f,-0.5f, 0.5f, 0,-1, 0, -0.5f,-0.5f,-0.5f, 0,-1, 0,  0.5f,-0.5f, 0.5f, 0,-1, 0, -0.5f,-0.5f, 0.5f, 0,-1, 0,
         0.5f,-0.5f,-0.5f, 1, 0, 0,  0.5f, 0.5f,-0.5f, 1, 0, 0,  0.5f, 0.5f, 0.5f, 1, 0, 0,  0.5f,-0.5f,-0.5f, 1, 0, 0,  0.5f, 0.5f, 0.5f, 1, 0, 0,  0.5f,-0.5f, 0.5f, 1, 0, 0,
        -0.5f,-0.5f,-0.5f,-1, 0, 0, -0.5f,-0.5f, 0.5f,-1, 0, 0, -0.5f, 0.5f, 0.5f,-1, 0, 0, -0.5f,-0.5f,-0.5f,-1, 0, 0, -0.5f, 0.5f, 0.5f,-1, 0, 0, -0.5f, 0.5f,-0.5f,-1, 0, 0,
    };
    constexpr float plane_vertices[] = {
        -10.0f, 0.0f,-10.0f, 0, 1, 0,  10.0f, 0.0f,-10.0f, 0, 1, 0,  10.0f, 0.0f, 10.0f, 0, 1, 0,
        -10.0f, 0.0f,-10.0f, 0, 1, 0,  10.0f, 0.0f, 10.0f, 0, 1, 0, -10.0f, 0.0f, 10.0f, 0, 1, 0,
    };
    const Mesh cube = CreateMesh(cube_vertices, 36);
    const Mesh plane = CreateMesh(plane_vertices, 6);
    const GLuint program = CreateProgram();
    glEnable(GL_DEPTH_TEST);

    bool running = true;
    bool looking = false;
    bool paused = false;
    bool single_step = false;
    Camera camera;
    auto previous_time = std::chrono::steady_clock::now();
    double accumulator = 0.0;
    while (running) {
        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            ImGui_ImplSDL3_ProcessEvent(&event);
            if (event.type == SDL_EVENT_QUIT) {
                running = false;
            }
            if (!ImGui::GetIO().WantCaptureMouse && event.type == SDL_EVENT_MOUSE_BUTTON_DOWN && event.button.button == SDL_BUTTON_RIGHT) {
                looking = true;
                SDL_SetWindowRelativeMouseMode(window, true);
            }
            if (event.type == SDL_EVENT_MOUSE_BUTTON_UP && event.button.button == SDL_BUTTON_RIGHT) {
                looking = false;
                SDL_SetWindowRelativeMouseMode(window, false);
            }
            if (looking && event.type == SDL_EVENT_MOUSE_MOTION) {
                camera.yaw += event.motion.xrel * 0.1f;
                camera.pitch = std::clamp(camera.pitch - event.motion.yrel * 0.1f, -89.0f, 89.0f);
            }
        }

        const auto now = std::chrono::steady_clock::now();
        const float frame_seconds = static_cast<float>(std::min(0.25, std::chrono::duration<double>(now - previous_time).count()));
        previous_time = now;
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplSDL3_NewFrame();
        ImGui::NewFrame();

        ImGui::Begin("Physics");
        if (ImGui::Button(paused ? "Resume" : "Pause")) {
            paused = !paused;
        }
        ImGui::SameLine();
        if (ImGui::Button("Single step")) {
            paused = true;
            single_step = true;
        }
        ImGui::SameLine();
        if (ImGui::Button("Reset")) {
            drone.Reset(bodies);
            imu_model.Reset(bodies.GetLinearVelocity(drone_id));
            simulation_time = 0.0;
            latest_imu_sample = imu_model.Sample(
                simulation_time,
                static_cast<float>(physics_step),
                bodies.GetRotation(drone_id),
                bodies.GetAngularVelocity(drone_id),
                bodies.GetLinearVelocity(drone_id),
                physics.GetGravity());
            accumulator = 0.0;
        }
        if (ImGui::DragFloat3("Gravity", gravity, 0.1f)) {
            physics.SetGravity(JPH::Vec3(gravity[0], gravity[1], gravity[2]));
            bodies.ActivateBody(drone_id);
        }
        float timestep_milliseconds = static_cast<float>(physics_step * 1000.0);
        if (ImGui::SliderFloat("Timestep (ms)", &timestep_milliseconds, 1.0f, 33.333f, "%.3f")) {
            physics_step = timestep_milliseconds / 1000.0;
            accumulator = std::min(accumulator, physics_step);
        }

        const JPH::RVec3 inspected_position = bodies.GetPosition(drone_id);
        const JPH::Quat inspected_rotation = bodies.GetRotation(drone_id);
        const JPH::Vec3 linear_velocity = bodies.GetLinearVelocity(drone_id);
        const JPH::Vec3 angular_velocity = bodies.GetAngularVelocity(drone_id);
        float mass = 0.0f;
        {
            JPH::BodyLockRead lock(physics.GetBodyLockInterface(), drone_id);
            if (lock.Succeeded()) {
                mass = 1.0f / lock.GetBody().GetMotionProperties()->GetInverseMass();
            }
        }
        ImGui::SeparatorText("Drone");
        ImGui::Text("Position: %.3f, %.3f, %.3f", inspected_position.GetX(), inspected_position.GetY(), inspected_position.GetZ());
        ImGui::Text("Rotation: %.3f, %.3f, %.3f, %.3f", inspected_rotation.GetX(), inspected_rotation.GetY(), inspected_rotation.GetZ(), inspected_rotation.GetW());
        ImGui::Text("Linear velocity: %.3f, %.3f, %.3f", linear_velocity.GetX(), linear_velocity.GetY(), linear_velocity.GetZ());
        ImGui::Text("Angular velocity: %.3f, %.3f, %.3f", angular_velocity.GetX(), angular_velocity.GetY(), angular_velocity.GetZ());
        ImGui::Text("Mass: %.3f", mass);
        ImGui::SeparatorText("Controller IMU");
        ImGui::Text("Timestamp: %.3f s", latest_imu_sample.timestamp_seconds);
        ImGui::Text(
            "Gyro (body, rad/s): %.3f, %.3f, %.3f",
            latest_imu_sample.body_gyro_rad_per_second.x,
            latest_imu_sample.body_gyro_rad_per_second.y,
            latest_imu_sample.body_gyro_rad_per_second.z);
        ImGui::Text(
            "Specific force (body, m/s^2): %.3f, %.3f, %.3f",
            latest_imu_sample.body_specific_force_meters_per_second_squared.x,
            latest_imu_sample.body_specific_force_meters_per_second_squared.y,
            latest_imu_sample.body_specific_force_meters_per_second_squared.z);
        ImGui::End();

        const bool *keys = SDL_GetKeyboardState(nullptr);
        const float movement_speed = (keys[SDL_SCANCODE_LSHIFT] || keys[SDL_SCANCODE_RSHIFT] ? 15.0f : 5.0f) * frame_seconds;
        const glm::vec3 forward = camera.Forward();
        const glm::vec3 horizontal_forward = glm::normalize(glm::vec3(forward.x, 0.0f, forward.z));
        const glm::vec3 right = glm::normalize(glm::cross(horizontal_forward, glm::vec3(0.0f, 1.0f, 0.0f)));
        if (!ImGui::GetIO().WantCaptureKeyboard) {
            if (keys[SDL_SCANCODE_W]) camera.position += horizontal_forward * movement_speed;
            if (keys[SDL_SCANCODE_S]) camera.position -= horizontal_forward * movement_speed;
            if (keys[SDL_SCANCODE_D]) camera.position += right * movement_speed;
            if (keys[SDL_SCANCODE_A]) camera.position -= right * movement_speed;
            if (keys[SDL_SCANCODE_SPACE]) camera.position.y += movement_speed;
            if (keys[SDL_SCANCODE_LCTRL] || keys[SDL_SCANCODE_RCTRL]) camera.position.y -= movement_speed;
        }

        const auto update_physics = [&] {
            latest_imu_sample = imu_model.Sample(
                simulation_time,
                static_cast<float>(physics_step),
                bodies.GetRotation(drone_id),
                bodies.GetAngularVelocity(drone_id),
                bodies.GetLinearVelocity(drone_id),
                physics.GetGravity());
            const ControllerInput controller_input{
                latest_imu_sample,
                static_cast<float>(physics_step),
            };
            UpdateDemoController(controller_input, target_drone);
            drone.SetMotorTargets(target_drone);
            drone.UpdateMotors();
            drone.ApplyForces(bodies);
            physics.Update(static_cast<float>(physics_step), 1, &temp_allocator, &job_system);
            simulation_time += physics_step;
        };
        if (single_step) {
            update_physics();
            accumulator = 0.0;
            single_step = false;
        } else if (!paused) {
            accumulator += frame_seconds;
            while (accumulator >= physics_step) {
                update_physics();
                accumulator -= physics_step;
            }
        }

        int width = 0;
        int height = 0;
        SDL_GetWindowSizeInPixels(window, &width, &height);
        glViewport(0, 0, width, height);
        glClearColor(0.08f, 0.10f, 0.14f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glUseProgram(program);

        const glm::mat4 projection = glm::perspective(glm::radians(50.0f), static_cast<float>(width) / static_cast<float>(height), 0.1f, 100.0f);
        const glm::mat4 view = glm::lookAt(camera.position, camera.position + camera.Forward(), glm::vec3(0.0f, 1.0f, 0.0f));
        glUniformMatrix4fv(glGetUniformLocation(program, "view_projection"), 1, GL_FALSE, glm::value_ptr(projection * view));
        DrawMesh(plane, program, glm::mat4(1.0f), glm::vec3(0.28f, 0.33f, 0.28f));

        const JPH::RVec3 position = bodies.GetPosition(drone_id);
        const JPH::Quat rotation = bodies.GetRotation(drone_id);
        const glm::mat4 transform = glm::translate(glm::mat4(1.0f), glm::vec3(position.GetX(), position.GetY(), position.GetZ()))
            * glm::mat4_cast(glm::quat(rotation.GetW(), rotation.GetX(), rotation.GetY(), rotation.GetZ()));
        DrawMesh(cube, program, transform * glm::scale(glm::mat4(1.0f), glm::vec3(0.5f, 0.16f, 0.5f)), glm::vec3(0.85f, 0.35f, 0.15f));
        for (const JPH::RVec3 &motor_position : drone.GetMotorWorldPositions(position, rotation)) {
            const glm::mat4 marker = glm::translate(glm::mat4(1.0f), glm::vec3(motor_position.GetX(), motor_position.GetY(), motor_position.GetZ()))
                * glm::scale(glm::mat4(1.0f), glm::vec3(0.05f));
            DrawMesh(cube, program, marker, glm::vec3(0.1f, 0.8f, 0.9f));
        }
        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        SDL_GL_SwapWindow(window);
    }

    SDL_SetWindowRelativeMouseMode(window, false);
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplSDL3_Shutdown();
    ImGui::DestroyContext();
    glDeleteProgram(program);
    SDL_GL_DestroyContext(context);
    SDL_DestroyWindow(window);
    SDL_Quit();
    JPH::UnregisterTypes();
    delete JPH::Factory::sInstance;
    JPH::Factory::sInstance = nullptr;
    return EXIT_SUCCESS;
}
