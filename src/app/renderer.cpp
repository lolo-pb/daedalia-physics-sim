#include "renderer.hpp"

#include <cstdio>
#include <cstdlib>

#include <glm/ext/matrix_clip_space.hpp>
#include <glm/ext/matrix_transform.hpp>
#include <glm/ext/quaternion_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

namespace {

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

} // namespace

Renderer::Mesh Renderer::CreateMesh(const float *vertices, GLuint vertex_count) {
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

void Renderer::DrawMesh(
    const Mesh &mesh,
    GLuint program,
    const glm::mat4 &model,
    const glm::vec3 &colour) {
    glUniformMatrix4fv(glGetUniformLocation(program, "model"), 1, GL_FALSE, glm::value_ptr(model));
    glUniform3fv(glGetUniformLocation(program, "colour"), 1, glm::value_ptr(colour));
    glBindVertexArray(mesh.vao);
    glDrawArrays(GL_TRIANGLES, 0, static_cast<GLsizei>(mesh.vertex_count));
}

Renderer::Renderer() {
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

    cube_ = CreateMesh(cube_vertices, 36);
    plane_ = CreateMesh(plane_vertices, 6);
    program_ = CreateProgram();
    glEnable(GL_DEPTH_TEST);
}

void Renderer::DrawScene(
    const DroneRenderState &drone,
    const glm::vec3 &camera_position,
    const glm::vec3 &camera_forward,
    int width,
    int height) const {
    glViewport(0, 0, width, height);
    glClearColor(0.08f, 0.10f, 0.14f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glUseProgram(program_);

    const glm::mat4 projection = glm::perspective(glm::radians(50.0f), static_cast<float>(width) / static_cast<float>(height), 0.1f, 100.0f);
    const glm::mat4 view = glm::lookAt(camera_position, camera_position + camera_forward, glm::vec3(0.0f, 1.0f, 0.0f));
    glUniformMatrix4fv(glGetUniformLocation(program_, "view_projection"), 1, GL_FALSE, glm::value_ptr(projection * view));
    DrawMesh(plane_, program_, glm::mat4(1.0f), glm::vec3(0.28f, 0.33f, 0.28f));

    const glm::mat4 transform = glm::translate(glm::mat4(1.0f), glm::vec3(drone.position.GetX(), drone.position.GetY(), drone.position.GetZ()))
        * glm::mat4_cast(glm::quat(drone.rotation.GetW(), drone.rotation.GetX(), drone.rotation.GetY(), drone.rotation.GetZ()));
    DrawMesh(cube_, program_, transform * glm::scale(glm::mat4(1.0f), glm::vec3(0.5f, 0.16f, 0.5f)), glm::vec3(0.85f, 0.35f, 0.15f));
    for (const JPH::RVec3 &motor_position : drone.motor_positions) {
        const glm::mat4 marker = glm::translate(glm::mat4(1.0f), glm::vec3(motor_position.GetX(), motor_position.GetY(), motor_position.GetZ()))
            * glm::scale(glm::mat4(1.0f), glm::vec3(0.05f));
        DrawMesh(cube_, program_, marker, glm::vec3(0.1f, 0.8f, 0.9f));
    }
}

void Renderer::Shutdown() {
    glDeleteProgram(program_);
}
