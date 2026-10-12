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

set(BUILD_PYTHON_MODULE OFF CACHE BOOL "" FORCE)
set(BUILD_DOCS OFF CACHE BOOL "" FORCE)
FetchContent_Declare(
    jsbsim
    URL https://github.com/JSBSim-Team/jsbsim/archive/refs/tags/v1.3.2.tar.gz
)
FetchContent_Populate(jsbsim)
add_subdirectory(${jsbsim_SOURCE_DIR} ${jsbsim_BINARY_DIR} EXCLUDE_FROM_ALL)

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

add_library(controller STATIC
    src/controllers/angle_mode_controller.cpp
    src/controllers/c172_direct_controller.cpp
    src/controllers/demo_controller.cpp
    src/controllers/horizon_mode_controller.cpp
    src/controllers/pid.cpp
    src/controllers/position_hold_controller.cpp
)
target_include_directories(controller PUBLIC
    src/controllers/include
    src/sensors/include
)

add_executable(daedalia
    src/app/application.cpp
    src/app/aircraft_selection_menu.cpp
    src/app/renderer.cpp
    src/app/simulation.cpp
    src/app/startup_animation.cpp
    src/app/ui.cpp
    src/main.cpp
    src/aircraft/aircraft_definition.cpp
    src/physics/jsbsim_backend.cpp
    src/sensors/ideal_barometer.cpp
    src/sensors/ideal_gps.cpp
    src/sensors/ideal_imu.cpp
    src/sensors/ideal_magnetometer.cpp
)
target_include_directories(daedalia PRIVATE
    src
    src/aircraft/include
)
target_link_libraries(daedalia PRIVATE controller SDL3::SDL3 libJSBSim glm::glm glad imgui OpenGL::GL)
target_include_directories(daedalia PRIVATE ${jsbsim_SOURCE_DIR}/src ${jsbsim_BINARY_DIR}/src)
# Keep model data beside the executable rather than relying on its working directory.
file(COPY ${jsbsim_SOURCE_DIR}/aircraft ${jsbsim_SOURCE_DIR}/engine ${jsbsim_SOURCE_DIR}/systems
    DESTINATION ${CMAKE_BINARY_DIR}/jsbsim-data)
target_compile_definitions(daedalia PRIVATE DAEDALIA_JSBSIM_DATA_DIR="${CMAKE_BINARY_DIR}/jsbsim-data")

if(BUILD_TESTING)
    add_test(
        NAME f450_backend_smoke_test
        COMMAND daedalia --smoke-test --aircraft f450
    )
    add_test(
        NAME c172_backend_smoke_test
        COMMAND daedalia --smoke-test --aircraft c172
    )
endif()
