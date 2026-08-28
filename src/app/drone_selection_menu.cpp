#include "drone_selection_menu.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <optional>
#include <string>

#include <glad/gl.h>
#include <SDL3/SDL.h>

#include <imgui.h>
#include <imgui_impl_opengl3.h>
#include <imgui_impl_sdl3.h>

namespace {

constexpr std::array<SDL_Scancode, 9> SelectionKeys = {
    SDL_SCANCODE_1,
    SDL_SCANCODE_2,
    SDL_SCANCODE_3,
    SDL_SCANCODE_4,
    SDL_SCANCODE_5,
    SDL_SCANCODE_6,
    SDL_SCANCODE_7,
    SDL_SCANCODE_8,
    SDL_SCANCODE_9,
};

void RestoreDepthTest(GLboolean depth_test_was_enabled) {
    if (depth_test_was_enabled) {
        glEnable(GL_DEPTH_TEST);
    }
}

std::optional<std::size_t> GetSelectionIndex(SDL_Scancode scancode) {
    for (std::size_t index = 0; index < SelectionKeys.size(); ++index) {
        if (scancode == SelectionKeys[index]) {
            return index;
        }
    }
    return std::nullopt;
}

} // namespace

std::optional<DroneType> SelectDroneFromMenu(SDL_Window *window) {
    const std::span<const DroneOption> options = GetAvailableDroneOptions();
    const GLboolean depth_test_was_enabled = glIsEnabled(GL_DEPTH_TEST);
    glDisable(GL_DEPTH_TEST);

    while (true) {
        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            ImGui_ImplSDL3_ProcessEvent(&event);
            if (event.type == SDL_EVENT_QUIT
                || (event.type == SDL_EVENT_KEY_DOWN
                    && event.key.scancode == SDL_SCANCODE_ESCAPE)) {
                RestoreDepthTest(depth_test_was_enabled);
                return std::nullopt;
            }
            if (event.type == SDL_EVENT_KEY_DOWN && !event.key.repeat) {
                const std::optional<std::size_t> index =
                    GetSelectionIndex(event.key.scancode);
                if (index && *index < options.size()) {
                    RestoreDepthTest(depth_test_was_enabled);
                    return options[*index].type;
                }
            }
        }

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplSDL3_NewFrame();
        ImGui::NewFrame();

        int framebuffer_width = 0;
        int framebuffer_height = 0;
        SDL_GetWindowSizeInPixels(
            window, &framebuffer_width, &framebuffer_height);
        glViewport(0, 0, framebuffer_width, framebuffer_height);
        glClearColor(0.08f, 0.10f, 0.14f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);

        const ImVec2 display_size = ImGui::GetIO().DisplaySize;
        const float menu_width = std::clamp(
            display_size.x * 0.42f, 320.0f, 520.0f);
        ImGui::SetNextWindowPos(
            ImVec2(display_size.x * 0.5f, display_size.y * 0.5f),
            ImGuiCond_Always,
            ImVec2(0.5f, 0.5f));
        ImGui::SetNextWindowSize(ImVec2(menu_width, 0.0f));
        constexpr ImGuiWindowFlags window_flags =
            ImGuiWindowFlags_NoCollapse
            | ImGuiWindowFlags_NoMove
            | ImGuiWindowFlags_NoResize
            | ImGuiWindowFlags_NoSavedSettings;
        ImGui::Begin("Select drone", nullptr, window_flags);
        ImGui::TextUnformatted("Choose an aircraft");
        ImGui::Spacing();

        std::optional<DroneType> selected_type;
        for (std::size_t index = 0; index < options.size(); ++index) {
            const DroneOption &option = options[index];
            ImGui::PushID(static_cast<int>(index));
            const std::string label = std::to_string(index + 1)
                + "  " + std::string(option.display_name);
            if (ImGui::Button(label.c_str(), ImVec2(-1.0f, 48.0f))) {
                selected_type = option.type;
            }
            ImGui::PopID();
        }

        ImGui::Spacing();
        ImGui::TextDisabled("Press a number key or click. Escape exits.");
        ImGui::End();

        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        SDL_GL_SwapWindow(window);

        if (selected_type) {
            RestoreDepthTest(depth_test_was_enabled);
            return selected_type;
        }
    }
}
