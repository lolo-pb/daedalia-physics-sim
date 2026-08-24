cmake_minimum_required(VERSION 3.24)

project(daedalia_physics_sim LANGUAGES C CXX)

include(CTest)

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

FetchContent_Declare(
    imgui
    URL https://github.com/ocornut/imgui/archive/refs/tags/v1.91.9b.tar.gz
)
FetchContent_Populate(imgui)
add_library(imgui STATIC
    ${imgui_SOURCE_DIR}/imgui.cpp
    ${imgui_SOURCE_DIR}/imgui_draw.cpp
    ${imgui_SOURCE_DIR}/imgui_tables.cpp
    ${imgui_SOURCE_DIR}/imgui_widgets.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_sdl3.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp
)
target_include_directories(imgui PUBLIC ${imgui_SOURCE_DIR} ${imgui_SOURCE_DIR}/backends)
target_compile_definitions(imgui PRIVATE IMGUI_IMPL_OPENGL_LOADER_GLAD2)
target_link_libraries(imgui PRIVATE SDL3::SDL3 glad OpenGL::GL)

find_package(OpenGL REQUIRED)

add_library(controller STATIC src/controller.cpp)
target_include_directories(controller PUBLIC src)

add_executable(daedalia
    src/main.cpp
    src/drone.cpp
    src/ideal_imu.cpp
)
target_link_libraries(daedalia PRIVATE controller SDL3::SDL3 Jolt glm::glm glad imgui OpenGL::GL)

if(BUILD_TESTING)
    add_executable(models_test
        tests/models_test.cpp
        src/ideal_imu.cpp
    )
    target_include_directories(models_test PRIVATE src)
    target_link_libraries(models_test PRIVATE controller Jolt)
    add_test(NAME models_test COMMAND models_test)
endif()
