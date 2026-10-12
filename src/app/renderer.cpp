#include "renderer.hpp"

#include <cstdio>
#include <vector>

#include <glm/ext/matrix_clip_space.hpp>
#include <glm/ext/matrix_transform.hpp>
#include <glm/ext/quaternion_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

namespace {

constexpr float FloorHalfExtent = 1000.0f;
constexpr int FloorGridCells = 64;

GLuint CompileShader(GLenum type, const char *source) {
    const GLuint shader = glCreateShader(type);
    if (shader == 0) {
        std::fprintf(stderr, "Shader creation failed\n");
        return 0;
    }
    glShaderSource(shader, 1, &source, nullptr);
    glCompileShader(shader);

    GLint compiled = GL_FALSE;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &compiled);
    if (compiled == GL_FALSE) {
        char log[1024];
        glGetShaderInfoLog(shader, sizeof(log), nullptr, log);
        std::fprintf(stderr, "Shader compilation failed: %s\n", log);
        glDeleteShader(shader);
        return 0;
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
        uniform bool curve_floor;
        out vec3 world_normal;
        out vec3 world_position;
        void main() {
            world_position = vec3(model * vec4(position, 1.0));
            world_normal = mat3(model) * normal;
            if (curve_floor) {
                world_position.y = -(world_position.x * world_position.x / (2.0 * 20925646.32546)
                    + world_position.z * world_position.z / (2.0 * 20855486.5951 * 20855486.5951 / 20925646.32546));
                world_normal = normalize(vec3(
                    world_position.x / 20925646.32546,
                    1.0,
                    world_position.z / (20855486.5951 * 20855486.5951 / 20925646.32546)));
            }
            gl_Position = view_projection * vec4(world_position, 1.0);
        }
    )";
    constexpr char fragment_source[] = R"(
        #version 450 core
        in vec3 world_normal;
        in vec3 world_position;
        uniform vec3 colour;
        uniform bool show_grid;
        out vec4 fragment_colour;
        void main() {
            vec3 surface_colour = colour;
            if (show_grid) {
                vec2 minor_width = fwidth(world_position.xz);
                vec2 minor_distance = abs(fract(world_position.xz - 0.5) - 0.5) / minor_width;
                float minor_line = 1.0 - min(min(minor_distance.x, minor_distance.y), 1.0);

                vec2 major_position = world_position.xz / 10.0;
                vec2 major_width = fwidth(major_position);
                vec2 major_distance = abs(fract(major_position - 0.5) - 0.5) / major_width;
                float major_line = 1.0 - min(min(major_distance.x, major_distance.y), 1.0);

                surface_colour = mix(surface_colour, vec3(0.16, 0.19, 0.16), minor_line * 0.55);
                surface_colour = mix(surface_colour, vec3(0.08, 0.10, 0.08), major_line * 0.85);
            }
            float light = max(dot(normalize(world_normal), normalize(vec3(0.4, 1.0, 0.3))), 0.0);
            fragment_colour = vec4(surface_colour * (0.25 + 0.75 * light), 1.0);
        }
    )";

    const GLuint program = glCreateProgram();
    if (program == 0) {
        std::fprintf(stderr, "Shader program creation failed\n");
        return 0;
    }
    const GLuint vertex_shader = CompileShader(GL_VERTEX_SHADER, vertex_source);
    if (vertex_shader == 0) {
        glDeleteProgram(program);
        return 0;
    }
    const GLuint fragment_shader = CompileShader(GL_FRAGMENT_SHADER, fragment_source);
    if (fragment_shader == 0) {
        glDeleteShader(vertex_shader);
        glDeleteProgram(program);
        return 0;
    }
    glAttachShader(program, vertex_shader);
    glAttachShader(program, fragment_shader);
    glLinkProgram(program);
    glDeleteShader(vertex_shader);
    glDeleteShader(fragment_shader);

    GLint linked = GL_FALSE;
    glGetProgramiv(program, GL_LINK_STATUS, &linked);
    if (linked == GL_FALSE) {
        char log[1024];
        glGetProgramInfoLog(program, sizeof(log), nullptr, log);
        std::fprintf(stderr, "Shader program linking failed: %s\n", log);
        glDeleteProgram(program);
        return 0;
    }
    return program;
}

} // namespace

Renderer::Mesh Renderer::CreateMesh(const float *vertices, GLuint vertex_count) {
    Mesh mesh{.vertex_count = vertex_count};
    glCreateVertexArrays(1, &mesh.vao);
    if (mesh.vao == 0) {
        return {};
    }
    glCreateBuffers(1, &mesh.vertex_buffer);
    if (mesh.vertex_buffer == 0) {
        DestroyMesh(mesh);
        return {};
    }
    glNamedBufferData(mesh.vertex_buffer, static_cast<GLsizeiptr>(vertex_count * 6 * sizeof(float)), vertices, GL_STATIC_DRAW);
    glVertexArrayVertexBuffer(mesh.vao, 0, mesh.vertex_buffer, 0, 6 * sizeof(float));
    glEnableVertexArrayAttrib(mesh.vao, 0);
    glEnableVertexArrayAttrib(mesh.vao, 1);
    glVertexArrayAttribFormat(mesh.vao, 0, 3, GL_FLOAT, GL_FALSE, 0);
    glVertexArrayAttribFormat(mesh.vao, 1, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float));
    glVertexArrayAttribBinding(mesh.vao, 0, 0);
    glVertexArrayAttribBinding(mesh.vao, 1, 0);
    return mesh;
}

void Renderer::DestroyMesh(Mesh &mesh) {
    if (mesh.vertex_buffer != 0) {
        glDeleteBuffers(1, &mesh.vertex_buffer);
    }
    if (mesh.vao != 0) {
        glDeleteVertexArrays(1, &mesh.vao);
    }
    mesh = {};
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

Renderer::~Renderer() {
    DestroyMesh(cube_);
    DestroyMesh(plane_);
    if (program_ != 0) {
        glDeleteProgram(program_);
    }
}

bool Renderer::Initialize() {
    constexpr float cube_vertices[] = {
        -0.5f,-0.5f, 0.5f, 0, 0, 1,  0.5f,-0.5f, 0.5f, 0, 0, 1,  0.5f, 0.5f, 0.5f, 0, 0, 1,  -0.5f,-0.5f, 0.5f, 0, 0, 1,  0.5f, 0.5f, 0.5f, 0, 0, 1,  -0.5f, 0.5f, 0.5f, 0, 0, 1,
        -0.5f,-0.5f,-0.5f, 0, 0,-1, -0.5f, 0.5f,-0.5f, 0, 0,-1,  0.5f, 0.5f,-0.5f, 0, 0,-1, -0.5f,-0.5f,-0.5f, 0, 0,-1,  0.5f, 0.5f,-0.5f, 0, 0,-1,  0.5f,-0.5f,-0.5f, 0, 0,-1,
        -0.5f, 0.5f,-0.5f, 0, 1, 0, -0.5f, 0.5f, 0.5f, 0, 1, 0,  0.5f, 0.5f, 0.5f, 0, 1, 0, -0.5f, 0.5f,-0.5f, 0, 1, 0,  0.5f, 0.5f, 0.5f, 0, 1, 0,  0.5f, 0.5f,-0.5f, 0, 1, 0,
        -0.5f,-0.5f,-0.5f, 0,-1, 0,  0.5f,-0.5f,-0.5f, 0,-1, 0,  0.5f,-0.5f, 0.5f, 0,-1, 0, -0.5f,-0.5f,-0.5f, 0,-1, 0,  0.5f,-0.5f, 0.5f, 0,-1, 0, -0.5f,-0.5f, 0.5f, 0,-1, 0,
         0.5f,-0.5f,-0.5f, 1, 0, 0,  0.5f, 0.5f,-0.5f, 1, 0, 0,  0.5f, 0.5f, 0.5f, 1, 0, 0,  0.5f,-0.5f,-0.5f, 1, 0, 0,  0.5f, 0.5f, 0.5f, 1, 0, 0,  0.5f,-0.5f, 0.5f, 1, 0, 0,
        -0.5f,-0.5f,-0.5f,-1, 0, 0, -0.5f,-0.5f, 0.5f,-1, 0, 0, -0.5f, 0.5f, 0.5f,-1, 0, 0, -0.5f,-0.5f,-0.5f,-1, 0, 0, -0.5f, 0.5f, 0.5f,-1, 0, 0, -0.5f, 0.5f,-0.5f,-1, 0, 0,
    };
    std::vector<float> plane_vertices;
    plane_vertices.reserve(FloorGridCells * FloorGridCells * 6 * 6);
    const auto append_floor_vertex = [&](float x, float z) {
        plane_vertices.insert(plane_vertices.end(), {x, 0.0f, z, 0.0f, 1.0f, 0.0f});
    };
    for (int z = 0; z < FloorGridCells; ++z) {
        const float near_z = -FloorHalfExtent + 2.0f * FloorHalfExtent * z / FloorGridCells;
        const float far_z = -FloorHalfExtent + 2.0f * FloorHalfExtent * (z + 1) / FloorGridCells;
        for (int x = 0; x < FloorGridCells; ++x) {
            const float left_x = -FloorHalfExtent + 2.0f * FloorHalfExtent * x / FloorGridCells;
            const float right_x = -FloorHalfExtent + 2.0f * FloorHalfExtent * (x + 1) / FloorGridCells;
            append_floor_vertex(left_x, near_z);
            append_floor_vertex(right_x, near_z);
            append_floor_vertex(right_x, far_z);
            append_floor_vertex(left_x, near_z);
            append_floor_vertex(right_x, far_z);
            append_floor_vertex(left_x, far_z);
        }
    }

    cube_ = CreateMesh(cube_vertices, 36);
    if (cube_.vao == 0) {
        std::fprintf(stderr, "Cube mesh creation failed\n");
        return false;
    }
    plane_ = CreateMesh(plane_vertices.data(), static_cast<GLuint>(plane_vertices.size() / 6));
    if (plane_.vao == 0) {
        std::fprintf(stderr, "Plane mesh creation failed\n");
        return false;
    }
    program_ = CreateProgram();
    if (program_ == 0) {
        return false;
    }
    glEnable(GL_DEPTH_TEST);
    return true;
}

void Renderer::DrawScene(
    const AircraftDefinition &definition,
    const AircraftRenderState &aircraft,
    const glm::vec3 &camera_position,
    const glm::vec3 &camera_forward,
    int width,
    int height) const {
    glViewport(0, 0, width, height);
    glClearColor(0.08f, 0.10f, 0.14f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glUseProgram(program_);

    const glm::mat4 projection = glm::perspective(glm::radians(50.0f), static_cast<float>(width) / static_cast<float>(height), 0.05f, 10000.0f);
    const glm::mat4 view = glm::lookAt(camera_position, camera_position + camera_forward, glm::vec3(0.0f, 1.0f, 0.0f));
    glUniformMatrix4fv(glGetUniformLocation(program_, "view_projection"), 1, GL_FALSE, glm::value_ptr(projection * view));
    glUniform1i(glGetUniformLocation(program_, "show_grid"), GL_TRUE);
    glUniform1i(glGetUniformLocation(program_, "curve_floor"), GL_TRUE);
    const glm::mat4 floor_transform = glm::translate(
        glm::mat4(1.0f), glm::vec3(camera_position.x, 0.0f, camera_position.z));
    DrawMesh(plane_, program_, floor_transform, glm::vec3(0.28f, 0.33f, 0.28f));
    glUniform1i(glGetUniformLocation(program_, "show_grid"), GL_FALSE);
    glUniform1i(glGetUniformLocation(program_, "curve_floor"), GL_FALSE);

    const glm::mat4 transform = glm::translate(glm::mat4(1), glm::vec3(aircraft.position_ft))
        * glm::mat4_cast(glm::quat(aircraft.rotation));
    for (const AircraftVisualPart &part : definition.visual_parts) {
        float angle = 0.0f;
        switch (part.motion) {
        case VisualPartMotion::Rotor: angle = static_cast<float>(aircraft.rotor_angles_rad[part.propulsion_index]); break;
        case VisualPartMotion::LeftAileron: angle = static_cast<float>(aircraft.left_aileron_rad); break;
        case VisualPartMotion::RightAileron: angle = static_cast<float>(aircraft.right_aileron_rad); break;
        case VisualPartMotion::Elevator: angle = static_cast<float>(aircraft.elevator_rad); break;
        case VisualPartMotion::Rudder: angle = -static_cast<float>(aircraft.rudder_rad); break;
        case VisualPartMotion::Flap: angle = static_cast<float>(aircraft.flap_rad); break;
        case VisualPartMotion::Fixed: break;
        }
        const glm::vec3 hinge(part.hinge_ft[0], part.hinge_ft[1], part.hinge_ft[2]);
        const glm::vec3 center(part.center_ft[0], part.center_ft[1], part.center_ft[2]);
        const glm::vec3 axis(part.axis[0], part.axis[1], part.axis[2]);
        const glm::vec3 size(part.size_ft[0], part.size_ft[1], part.size_ft[2]);
        const glm::vec3 color(part.color[0], part.color[1], part.color[2]);
        const glm::mat4 model = transform * glm::translate(glm::mat4(1), hinge)
            * glm::rotate(glm::mat4(1), angle, axis)
            * glm::translate(glm::mat4(1), center - hinge)
            * glm::rotate(glm::mat4(1), part.base_yaw_rad, glm::vec3(0,1,0))
            * glm::scale(glm::mat4(1), size);
        DrawMesh(cube_, program_, model, color);
    }
}
