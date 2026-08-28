#include "startup_animation.hpp"

#include <algorithm>
#include <array>
#include <cfloat>
#include <chrono>
#include <cmath>

#include <glad/gl.h>
#include <SDL3/SDL.h>

#include <imgui.h>
#include <imgui_impl_opengl3.h>
#include <imgui_impl_sdl3.h>

bool PlayStartupAnimation(SDL_Window *window) {
    constexpr float duration_seconds = 4.6f;
    constexpr std::array<unsigned, 18> flicker_pattern = {
        1, 0, 2, 0, 1, 2, 0, 1, 0, 2, 1, 0, 2, 1, 2, 0, 3, 3,
    };

    const GLboolean depth_test_was_enabled = glIsEnabled(GL_DEPTH_TEST);
    glDisable(GL_DEPTH_TEST);
    const auto started_at = std::chrono::steady_clock::now();

    while (true) {
        const float elapsed = std::chrono::duration<float>(
            std::chrono::steady_clock::now() - started_at).count();
        if (elapsed >= duration_seconds) {
            break;
        }

        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            ImGui_ImplSDL3_ProcessEvent(&event);
            if (event.type == SDL_EVENT_QUIT) {
                if (depth_test_was_enabled) {
                    glEnable(GL_DEPTH_TEST);
                }
                return false;
            }
            if (event.type == SDL_EVENT_KEY_DOWN
                || event.type == SDL_EVENT_MOUSE_BUTTON_DOWN) {
                if (depth_test_was_enabled) {
                    glEnable(GL_DEPTH_TEST);
                }
                return true;
            }
        }

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplSDL3_NewFrame();
        ImGui::NewFrame();

        int framebuffer_width = 0;
        int framebuffer_height = 0;
        SDL_GetWindowSizeInPixels(window, &framebuffer_width, &framebuffer_height);
        glViewport(0, 0, framebuffer_width, framebuffer_height);
        glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);

        const ImVec2 display_size = ImGui::GetIO().DisplaySize;
        const float scale = std::min(display_size.x, display_size.y);
        const float mark_radius = std::clamp(scale * 0.085f, 42.0f, 72.0f);
        const float text_font_size = std::clamp(scale * 0.045f, 25.0f, 38.0f);
        const ImVec2 mark_center(display_size.x * 0.5f, display_size.y * 0.41f);
        ImDrawList *draw_list = ImGui::GetBackgroundDrawList();

        float composition_alpha = 1.0f;
        if (elapsed > 4.25f) {
            composition_alpha = std::clamp((duration_seconds - elapsed) / 0.35f, 0.0f, 1.0f);
        }

        auto white = [](float alpha) {
            return IM_COL32(255, 255, 255, static_cast<int>(255.0f * std::clamp(alpha, 0.0f, 1.0f)));
        };

        auto draw_mark = [&](bool draw_diamond, bool draw_cross, float alpha, ImVec2 offset = ImVec2()) {
            const ImVec2 center(mark_center.x + offset.x, mark_center.y + offset.y);
            if (draw_diamond) {
                const std::array<ImVec2, 4> diamond = {
                    ImVec2(center.x, center.y - mark_radius),
                    ImVec2(center.x + mark_radius, center.y),
                    ImVec2(center.x, center.y + mark_radius),
                    ImVec2(center.x - mark_radius, center.y),
                };
                draw_list->AddPolyline(diamond.data(), diamond.size(), white(alpha), ImDrawFlags_Closed, 1.5f);
            }
            if (draw_cross) {
                const float half_width = mark_radius * 0.54f;
                const float half_height = mark_radius * 0.90f;
                draw_list->AddLine(
                    ImVec2(center.x - half_width, center.y - half_height),
                    ImVec2(center.x + half_width, center.y + half_height),
                    white(alpha),
                    1.5f);
                draw_list->AddLine(
                    ImVec2(center.x - half_width, center.y + half_height),
                    ImVec2(center.x + half_width, center.y - half_height),
                    white(alpha),
                    1.5f);
            }
        };

        auto draw_centered_text = [&](const char *text, float font_size, float y, float alpha) {
            ImFont *font = ImGui::GetFont();
            const ImVec2 text_size = font->CalcTextSizeA(font_size, FLT_MAX, 0.0f, text);
            draw_list->AddText(
                font,
                font_size,
                ImVec2((display_size.x - text_size.x) * 0.5f, y),
                white(alpha),
                text);
        };

        auto draw_block_reveal = [&](const char *text, float font_size, ImVec2 text_position, float progress) {
            ImFont *font = ImGui::GetFont();
            const ImVec2 text_size = font->CalcTextSizeA(font_size, FLT_MAX, 0.0f, text);
            const ImVec2 block_max(text_position.x + text_size.x, text_position.y + text_size.y);
            if (progress <= 0.0f) {
                return;
            }
            if (progress < 0.22f) {
                const float block_alpha = std::sin(progress / 0.22f * 3.14159265f);
                draw_list->AddRectFilled(text_position, block_max, white(block_alpha * composition_alpha));
                return;
            }

            const float reveal = std::clamp((progress - 0.22f) / 0.48f, 0.0f, 1.0f);
            const float reveal_x = text_position.x + text_size.x * reveal;
            draw_list->PushClipRect(text_position, ImVec2(reveal_x, block_max.y), true);
            draw_list->AddText(font, font_size, text_position, white(composition_alpha), text);
            draw_list->PopClipRect();
            if (reveal < 1.0f) {
                draw_list->AddRectFilled(
                    ImVec2(std::max(text_position.x, reveal_x - 3.0f), text_position.y),
                    ImVec2(reveal_x, block_max.y),
                    white(composition_alpha));
            }
        };

        if (elapsed < 1.35f) {
            const int flicker_index = std::min(
                static_cast<int>(elapsed / 0.075f),
                static_cast<int>(flicker_pattern.size() - 1));
            const unsigned state = flicker_pattern[flicker_index];
            const float jitter_x = (flicker_index % 3 - 1) * 1.2f;
            const float jitter_y = ((flicker_index * 2) % 3 - 1) * 0.8f;
            draw_mark((state & 1U) != 0, (state & 2U) != 0, 0.86f, ImVec2(jitter_x, jitter_y));
        } else if (elapsed < 2.30f) {
            if (elapsed < 1.48f) {
                draw_mark(true, true, composition_alpha);
            }
            const int flash = static_cast<int>((elapsed - 1.48f) / 0.085f);
            const bool text_visible = elapsed > 1.48f && (flash % 3 != 1 || elapsed > 2.12f);
            if (text_visible) {
                draw_centered_text(
                    "I.M.D.E.D",
                    text_font_size,
                    (display_size.y - text_font_size) * 0.5f,
                    composition_alpha);
            }
        } else if (elapsed >= 2.42f) {
            const float reveal_progress = std::clamp((elapsed - 2.42f) / 1.05f, 0.0f, 1.0f);
            ImFont *font = ImGui::GetFont();
            const ImVec2 title_size = font->CalcTextSizeA(text_font_size, FLT_MAX, 0.0f, "Daedalia");
            const ImVec2 subtitle_size = font->CalcTextSizeA(text_font_size, FLT_MAX, 0.0f, "Drone Sim");
            const float line_spacing = scale * 0.018f;
            const float left = (display_size.x - std::max(title_size.x, subtitle_size.x)) * 0.5f;
            const float title_y = (display_size.y - title_size.y - line_spacing - subtitle_size.y) * 0.5f;
            draw_block_reveal("Daedalia", text_font_size, ImVec2(left, title_y), reveal_progress);
            draw_block_reveal(
                "Drone Sim",
                text_font_size,
                ImVec2(left, title_y + title_size.y + line_spacing),
                std::clamp(reveal_progress - 0.12f, 0.0f, 1.0f));
        }

        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        SDL_GL_SwapWindow(window);
    }

    if (depth_test_was_enabled) {
        glEnable(GL_DEPTH_TEST);
    }
    return true;
}
