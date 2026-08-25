#pragma once

#include <glad/gl.h>

#include <glm/mat4x4.hpp>
#include <glm/vec3.hpp>

#include "simulation.hpp"

class Renderer {
public:
    Renderer() = default;
    ~Renderer();

    Renderer(const Renderer &) = delete;
    Renderer &operator=(const Renderer &) = delete;

    bool Initialize();

    void DrawScene(
        const DroneRenderState &drone,
        const glm::vec3 &camera_position,
        const glm::vec3 &camera_forward,
        int width,
        int height) const;

private:
    struct Mesh {
        GLuint vao = 0;
        GLuint vertex_buffer = 0;
        GLuint vertex_count = 0;
    };

    static Mesh CreateMesh(const float *vertices, GLuint vertex_count);
    static void DestroyMesh(Mesh &mesh);
    static void DrawMesh(
        const Mesh &mesh,
        GLuint program,
        const glm::mat4 &model,
        const glm::vec3 &colour);

    Mesh cube_;
    Mesh plane_;
    GLuint program_ = 0;
};
