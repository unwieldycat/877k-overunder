#pragma once
#include "main.h"

namespace alt_odom {

/**
 * @brief alternate position tracking function
 *
 */
void track_pos();

/**
 * @brief gets current x coordinate
 *
 * @return inch_t
 */
inch_t x_coord();

/**
 * @brief gets current y coordinate
 *
 * @return inch_t
 */
inch_t y_coord();

} // namespace alt_odom