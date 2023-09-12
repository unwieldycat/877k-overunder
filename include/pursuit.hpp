#pragma once
#include "devices.hpp"
#include "main.h"

/**
 * @brief Create vector of pairs to store points for the path with initial point 0, 0
 *
 */
extern std::vector<std::pair<foot_t, foot_t>> points;

namespace pursuit {

/**
 * @brief adds a point to the end of the points for the path
 *
 * @param x_ft x coordinate in feet
 * @param y_ft y coordinate in feet
 */
void add_point(foot_t x_ft, foot_t y_ft);

/**
 * @brief Begins moving the robot following all the points stored previous to the running of this
 * function, clears the points after running
 *
 * @param lookahead_Distance How far away the point the robot is "Chasing" is
 * @param voltage_constant How fast the robot will be moving forward if it is traveling on a
 * straight path
 * @param lowest_x Bottom left x coordinate in feet of the restricted area
 * @param lowest_y Bottom left y coordinate in feet of the restricted area
 * @param highest_x Bottom right x coordinate in feet of the restricted area
 * @param highest_y Bottom right y coordinate in feet of the restricted area
 */
void pursuit(
    foot_t lookahead_distance, int voltage_constant, foot_t lowest_x, foot_t lowest_y,
    foot_t highest_x, foot_t highest_y
);

} // namespace pursuit