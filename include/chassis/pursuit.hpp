#pragma once
#include "point.hpp"
#include "units.h"
#include <vector>

namespace chassis {

/**
 * @brief adds a point to the end of the points for the path
 *
 * @param x_ft x coordinate in feet
 * @param y_ft y coordinate in feet
 * @param curvature curvature |<x'(t), y'(t)> cross <x"(t), y"(t)>|/|<x'(t), y'(t)>|^3
 * @param wings Wings to be opened (n, l, r, b)
 */
void add_point(
    foot_t x_ft, foot_t y_ft, units::dimensionless::scalar_t curvature, bool left_wing,
    bool right_wing
);

/**
 * @brief Begins moving the robot following all the points stored previous to the running of this
 * function, clears the points after running
 *
 * @param file_path File path to the file with the pursuit path
 * @param backwards determines if the bot should move backwards for this pursuit cycle
 */
void pursuit(std::string file_path, bool backwards = false);

/**
 * @brief Finds the lookahead distance
 *
 * @param curvature curvature of the function around the point
 * @param max maximum lookahead distance
 * @param min minimum lookahead distance
 * @return foot_t
 */
foot_t calculate_lookahead(double curvature, foot_t max, foot_t min);

} // namespace chassis