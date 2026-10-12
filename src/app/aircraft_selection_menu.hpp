#pragma once

#include <optional>

#include "aircraft/aircraft_definition.hpp"

struct SDL_Window;

std::optional<AircraftType> SelectAircraftFromMenu(SDL_Window *window);
