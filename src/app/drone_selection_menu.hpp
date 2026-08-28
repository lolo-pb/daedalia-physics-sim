#pragma once

#include <optional>

#include "drones/drone_definition.hpp"

struct SDL_Window;

std::optional<DroneType> SelectDroneFromMenu(SDL_Window *window);
