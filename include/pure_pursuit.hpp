#pragma once
#include "devices.hpp"
#include "main.h"

namespace pursuit {

void add_point(double x_ft, double y_ft);

void pursuit(
    int lookahead_distance, int voltage_constant, int lowest_x, int lowest_y, int highest_x,
    int highest_y
);

} // namespace pursuit