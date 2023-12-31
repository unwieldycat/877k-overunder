#pragma once
#include "point.hpp"
#include <vector>

namespace chassis {

/**
 * @brief adds a point to the end of the points for the path
 *
 * @param x_ft x coordinate in feet
 * @param y_ft y coordinate in feet
 * @param curvature curvature |<x'(t), y'(t)> cross <x"(t), y"(t)>|/|<x'(t), y'(t)>|^3
 */
void add_point(foot_t x_ft, foot_t y_ft, units::dimensionless::scalar_t curvature);

/**
 * @brief Begins moving the robot following all the points stored previous to the running of this
 * function, clears the points after running
 *
 * @param backwards determines if the bot should move backwards for this pursuit cycle
 */
void pursuit(bool backwards = false);

} // namespace chassis