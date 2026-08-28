#include "application.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <optional>

#include <glad/gl.h>
#include <SDL3/SDL.h>

#include <imgui.h>
#include <imgui_impl_opengl3.h>
#include <imgui_impl_sdl3.h>

#include <glm/ext/matrix_transform.hpp>

#include "drone_selection_menu.hpp"
#include "renderer.hpp"
#include "simulation.hpp"
#include "startup_animation.hpp"
#include "ui.hpp"

namespace {

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

    void LookAt(const glm::vec3 &target) {
        const glm::vec3 offset = target - position;
        if (glm::length(offset) == 0.0f) {
            return;
        }

        const glm::vec3 direction = glm::normalize(offset);
        yaw = glm::degrees(std::atan2(direction.z, direction.x));
        pitch = glm::degrees(std::asin(std::clamp(direction.y, -1.0f, 1.0f)));
    }

    void OrbitAround(const glm::vec3 &target, float yaw_delta, float pitch_delta) {
        const float distance = glm::length(position - target);
        if (distance == 0.0f) {
            return;
        }

        yaw += yaw_delta;
        pitch = std::clamp(pitch + pitch_delta, -89.0f, 89.0f);
        position = target - Forward() * distance;
    }
};

glm::vec3 ToGlm(const JPH::RVec3 &vector) {
    return glm::vec3(vector.GetX(), vector.GetY(), vector.GetZ());
}

class ApplicationResources {
public:
    ~ApplicationResources() {
        if (window_ != nullptr) {
            SDL_SetWindowRelativeMouseMode(window_, false);
        }
        if (imgui_opengl_initialized_) {
            ImGui_ImplOpenGL3_Shutdown();
        }
        if (imgui_sdl_initialized_) {
            ImGui_ImplSDL3_Shutdown();
        }
        if (imgui_context_created_) {
            ImGui::DestroyContext();
        }
        renderer_.reset();
        if (context_ != nullptr) {
            SDL_GL_DestroyContext(context_);
        }
        if (window_ != nullptr) {
            SDL_DestroyWindow(window_);
        }
        if (sdl_initialized_) {
            SDL_Quit();
        }
    }

    bool Initialize() {
        if (!SDL_Init(SDL_INIT_VIDEO)) {
            std::fprintf(stderr, "SDL initialization failed: %s\n", SDL_GetError());
            return false;
        }
        sdl_initialized_ = true;

        SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 4);
        SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 5);
        SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
        window_ = SDL_CreateWindow("Daedalia Physics Sim", 1280, 720, SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE);
        if (window_ == nullptr) {
            std::fprintf(stderr, "Window creation failed: %s\n", SDL_GetError());
            return false;
        }

        context_ = SDL_GL_CreateContext(window_);
        if (context_ == nullptr || !gladLoadGL(reinterpret_cast<GLADloadfunc>(SDL_GL_GetProcAddress))) {
            std::fprintf(stderr, "OpenGL initialization failed: %s\n", SDL_GetError());
            return false;
        }
        SDL_GL_SetSwapInterval(1);

        IMGUI_CHECKVERSION();
        imgui_context_created_ = ImGui::CreateContext() != nullptr;
        if (!imgui_context_created_) {
            std::fprintf(stderr, "ImGui context creation failed\n");
            return false;
        }
        ImGui::StyleColorsDark();
        imgui_sdl_initialized_ = ImGui_ImplSDL3_InitForOpenGL(window_, context_);
        if (!imgui_sdl_initialized_) {
            std::fprintf(stderr, "ImGui SDL initialization failed\n");
            return false;
        }
        imgui_opengl_initialized_ = ImGui_ImplOpenGL3_Init("#version 450");
        if (!imgui_opengl_initialized_) {
            std::fprintf(stderr, "ImGui OpenGL initialization failed\n");
            return false;
        }

        renderer_.emplace();
        if (!renderer_->Initialize()) {
            std::fprintf(stderr, "Renderer initialization failed\n");
            return false;
        }
        return true;
    }

    SDL_Window *Window() const {
        return window_;
    }

    Renderer &GetRenderer() {
        return *renderer_;
    }

private:
    bool sdl_initialized_ = false;
    bool imgui_context_created_ = false;
    bool imgui_sdl_initialized_ = false;
    bool imgui_opengl_initialized_ = false;
    SDL_Window *window_ = nullptr;
    SDL_GLContext context_ = nullptr;
    std::optional<Renderer> renderer_;
};

} // namespace

int RunInteractiveApplication(std::optional<DroneType> selected_drone) {
    ApplicationResources resources;
    if (!resources.Initialize()) {
        return EXIT_FAILURE;
    }
    SDL_Window *window = resources.Window();
    if (!PlayStartupAnimation(window)) return EXIT_SUCCESS;
    if (!selected_drone) {
        selected_drone = SelectDroneFromMenu(window);
        if (!selected_drone) {
            return EXIT_SUCCESS;
        }
    }
    Simulation simulation(CreateDroneDefinition(*selected_drone));
    Renderer &renderer = resources.GetRenderer();
    bool running = true;
    bool looking = false;
    bool paused = false;
    bool single_step = false;
    bool follow_drone = false;
    Camera camera;
    glm::vec3 previous_drone_position = ToGlm(simulation.GetDronePosition());
    auto previous_time = std::chrono::steady_clock::now();
    double accumulator = 0.0;
    while (running) {
        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            ImGui_ImplSDL3_ProcessEvent(&event);
            if (event.type == SDL_EVENT_QUIT) {
                running = false;
            }
            if (event.type == SDL_EVENT_KEY_DOWN && !event.key.repeat) {
                switch (event.key.scancode) {
                case SDL_SCANCODE_1: simulation.SelectController(1); break;
                case SDL_SCANCODE_2: simulation.SelectController(2); break;
                case SDL_SCANCODE_3: simulation.SelectController(3); break;
                case SDL_SCANCODE_4: simulation.SelectController(4); break;
                case SDL_SCANCODE_5: simulation.SelectController(5); break;
                case SDL_SCANCODE_6: simulation.SelectController(6); break;
                case SDL_SCANCODE_7: simulation.SelectController(7); break;
                case SDL_SCANCODE_8: simulation.SelectController(8); break;
                case SDL_SCANCODE_9: simulation.SelectController(9); break;
                case SDL_SCANCODE_0: simulation.SelectController(0); break;
                default: break;
                }
            }
            if (event.type == SDL_EVENT_MOUSE_BUTTON_DOWN && event.button.button == SDL_BUTTON_RIGHT) {
                looking = true;
                SDL_SetWindowRelativeMouseMode(window, true);
            }
            if (event.type == SDL_EVENT_MOUSE_BUTTON_UP && event.button.button == SDL_BUTTON_RIGHT) {
                looking = false;
                SDL_SetWindowRelativeMouseMode(window, false);
            }
            if (looking && event.type == SDL_EVENT_MOUSE_MOTION) {
                if (follow_drone) {
                    camera.OrbitAround(
                        ToGlm(simulation.GetDronePosition()),
                        event.motion.xrel * 0.1f,
                        -event.motion.yrel * 0.1f);
                } else {
                    camera.yaw += event.motion.xrel * 0.1f;
                    camera.pitch = std::clamp(camera.pitch - event.motion.yrel * 0.1f, -89.0f, 89.0f);
                }
            }
        }

        const auto now = std::chrono::steady_clock::now();
        const float frame_seconds = static_cast<float>(std::min(0.25, std::chrono::duration<double>(now - previous_time).count()));
        previous_time = now;
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplSDL3_NewFrame();
        ImGui::NewFrame();

        const PhysicsPanelResult panel_result = DrawPhysicsPanel(simulation, paused, single_step, follow_drone);
        if (panel_result.reset) {
            accumulator = 0.0;
        }
        if (panel_result.physics_frequency_changed) {
            accumulator = std::min(accumulator, simulation.GetPhysicsStepSeconds());
        }
        if (panel_result.follow_drone_enabled) {
            previous_drone_position = ToGlm(simulation.GetDronePosition());
            camera.LookAt(previous_drone_position);
        }
        DrawSensorsPanel(simulation);

        const bool *keys = SDL_GetKeyboardState(nullptr);
        const float movement_speed = (keys[SDL_SCANCODE_LSHIFT] || keys[SDL_SCANCODE_RSHIFT] ? 15.0f : 5.0f) * frame_seconds;
        const glm::vec3 forward = camera.Forward();
        const glm::vec3 horizontal_forward = glm::normalize(glm::vec3(forward.x, 0.0f, forward.z));
        const glm::vec3 right = glm::normalize(glm::cross(horizontal_forward, glm::vec3(0.0f, 1.0f, 0.0f)));
        const bool camera_accepts_keyboard = looking
            || (simulation.GetActiveController() == FlightController::Demo && !ImGui::GetIO().WantCaptureKeyboard);
        if (camera_accepts_keyboard) {
            if (keys[SDL_SCANCODE_W]) camera.position += horizontal_forward * movement_speed;
            if (keys[SDL_SCANCODE_S]) camera.position -= horizontal_forward * movement_speed;
            if (keys[SDL_SCANCODE_D]) camera.position += right * movement_speed;
            if (keys[SDL_SCANCODE_A]) camera.position -= right * movement_speed;
            if (keys[SDL_SCANCODE_SPACE]) camera.position.y += movement_speed;
            if (keys[SDL_SCANCODE_LCTRL] || keys[SDL_SCANCODE_RCTRL]) camera.position.y -= movement_speed;
        }
        if (follow_drone) {
            camera.LookAt(ToGlm(simulation.GetDronePosition()));
        }

        ControllerKeys controller_keys;
        if (!looking && !ImGui::GetIO().WantCaptureKeyboard) {
            controller_keys.w = keys[SDL_SCANCODE_W];
            controller_keys.a = keys[SDL_SCANCODE_A];
            controller_keys.s = keys[SDL_SCANCODE_S];
            controller_keys.d = keys[SDL_SCANCODE_D];
            controller_keys.q = keys[SDL_SCANCODE_Q];
            controller_keys.e = keys[SDL_SCANCODE_E];
            controller_keys.r = keys[SDL_SCANCODE_R];
            controller_keys.f = keys[SDL_SCANCODE_F];
            controller_keys.x = keys[SDL_SCANCODE_X];
        }

        if (single_step) {
            simulation.Step(controller_keys);
            accumulator = 0.0;
            single_step = false;
        } else if (!paused) {
            accumulator += frame_seconds;
            while (accumulator >= simulation.GetPhysicsStepSeconds()) {
                simulation.Step(controller_keys);
                accumulator -= simulation.GetPhysicsStepSeconds();
            }
        }

        const glm::vec3 current_drone_position = ToGlm(simulation.GetDronePosition());
        if (follow_drone) {
            camera.position += current_drone_position - previous_drone_position;
            camera.LookAt(current_drone_position);
        }
        previous_drone_position = current_drone_position;

        int width = 0;
        int height = 0;
        SDL_GetWindowSizeInPixels(window, &width, &height);
        renderer.DrawScene(simulation.GetDroneRenderState(), camera.position, camera.Forward(), width, height);
        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        SDL_GL_SwapWindow(window);
    }

    return EXIT_SUCCESS;
}
