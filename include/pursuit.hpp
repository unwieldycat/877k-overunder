#pragma once
#include "devices.hpp"
#include "main.h"

extern std::vector<std::pair<foot_t, foot_t>> points;

namespace pursuit {

void add_point(foot_t x_ft, foot_t y_ft);

void pursuit(
    foot_t lookahead_distance, int voltage_constant, foot_t lowest_x, foot_t lowest_y,
    foot_t highest_x, foot_t highest_y
);

} // namespace pursuit