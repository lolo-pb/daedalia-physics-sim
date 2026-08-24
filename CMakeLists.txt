cmake_minimum_required(VERSION 3.24)

project(daedalia_physics_sim LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(FetchContent)

set(SDL_TEST_LIBRARY OFF CACHE BOOL "" FORCE)
FetchContent_Declare(
    SDL3
    URL https://github.com/libsdl-org/SDL/archive/refs/tags/release-3.2.8.tar.gz
)
FetchContent_MakeAvailable(SDL3)

FetchContent_Declare(
    JoltPhysics
    URL https://github.com/jrouwe/JoltPhysics/archive/refs/tags/v5.3.0.tar.gz
    SOURCE_SUBDIR Build
)
FetchContent_MakeAvailable(JoltPhysics)

FetchContent_Declare(
    glm
    URL https://github.com/g-truc/glm/archive/refs/tags/1.0.1.tar.gz
)
FetchContent_MakeAvailable(glm)

FetchContent_Declare(
    glad
    URL https://github.com/Dav1dde/glad/archive/refs/tags/v2.0.8.tar.gz
)
FetchContent_MakeAvailable(glad)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
set(GLAD_OUTPUT_DIR "${CMAKE_BINARY_DIR}/generated/glad")
execute_process(
    COMMAND ${CMAKE_COMMAND} -E env "PYTHONPATH=${glad_SOURCE_DIR}"
        ${Python3_EXECUTABLE} -m glad --out-path ${GLAD_OUTPUT_DIR}
        --api gl:core=4.5 --reproducible c --loader
    RESULT_VARIABLE GLAD_RESULT
)
if (NOT GLAD_RESULT EQUAL 0)
    message(FATAL_ERROR "GLAD generation failed")
endif()
add_library(glad STATIC "${GLAD_OUTPUT_DIR}/src/gl.c")
target_include_directories(glad PUBLIC "${GLAD_OUTPUT_DIR}/include")

find_package(OpenGL REQUIRED)

add_executable(daedalia src/main.cpp)
target_link_libraries(daedalia PRIVATE SDL3::SDL3 Jolt glm::glm glad OpenGL::GL)
