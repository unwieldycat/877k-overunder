#pragma once
#include "point.hpp"
#include <vector>

namespace chassis {
namespace pursuit {

/**
 * @brief Create vector of pairs to store points for the path with initial point 0, 0
 *
 */
extern std::vector<Point> points;

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
 */
void pursuit();

/**
 * @brief Toggles the wings at a certain point or in a certain area
 *
 * @param side Side to change (L, R, or B)
 */
void wing(char side);

} // namespace pursuit
} // namespace chassis