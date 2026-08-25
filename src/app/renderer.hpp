#pragma once

#include <glad/gl.h>

#include <glm/mat4x4.hpp>
#include <glm/vec3.hpp>

#include "simulation.hpp"

class Renderer {
public:
    Renderer();

    void DrawScene(
        const DroneRenderState &drone,
        const glm::vec3 &camera_position,
        const glm::vec3 &camera_forward,
        int width,
        int height) const;
    void Shutdown();

private:
    struct Mesh {
        GLuint vao = 0;
        GLuint vertex_count = 0;
    };

    static Mesh CreateMesh(const float *vertices, GLuint vertex_count);
    static void DrawMesh(
        const Mesh &mesh,
        GLuint program,
        const glm::mat4 &model,
        const glm::vec3 &colour);

    Mesh cube_;
    Mesh plane_;
    GLuint program_ = 0;
};
