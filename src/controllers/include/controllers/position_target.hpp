#pragma once

#include <array>

struct PositionTarget {
    double north_ft, east_ft, height_ft;
};

struct TimedPositionTarget {
    int tick;
    PositionTarget target; // Feet from launch, positive up.
};

inline constexpr std::array<TimedPositionTarget, 4> BaselineTrajectory{{
    {0,    {0, 0, 10}},
    {900,  {0, 0, 15}},
    {1800, {10, 0, 15}},
    {2700, {0, 5, 10}},
}};
