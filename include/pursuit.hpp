#pragma once
#include "devices.hpp"
#include "main.h"

extern std::vector<std::pair<double, double>> points;

namespace pursuit {

void add_point(double x_ft, double y_ft);

void pursuit(
    double lookahead_distance, int voltage_constant, int lowest_x, int lowest_y, int highest_x,
    int highest_y
);

} // namespace pursuit