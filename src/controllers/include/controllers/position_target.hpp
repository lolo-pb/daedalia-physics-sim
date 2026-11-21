#pragma once

#include <array>

struct PositionTarget {
	double north_ft, east_ft, height_ft;
};

struct TimedPositionTarget {
	int tick;
	PositionTarget target; // Feet from launch, positive up.
};

inline constexpr std::array<TimedPositionTarget, 8> BaselineTrajectory{{
		{0, {0, 0, 10}},
		{600, {10, 0, 15}},
		{900, {-10, 8, 8}},
		{1200, {8, -10, 16}},
		{1500, {-8, -8, 9}},
		{1800, {10, 10, 14}},
		{2100, {-10, 0, 8}},
		{2400, {0, 0, 10}},
}};
