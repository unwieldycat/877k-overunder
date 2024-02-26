#pragma once
#include "main.h"

namespace odom {

/**
 * @brief Initialize odometry
 */
void initialize();

/**
 * @brief Position tracking function
 */
[[noreturn]] void track_position();

/**
 * @brief Calibrate odometry
 *
 * @param robot_x Start position x
 * @param robot_y Start position y
 * @param heading Starting heading
 */
void calibrate(inch_t robot_x, inch_t robot_y, degree_t heading);

/**
 * @brief Calibrate with GPS
 *
 * @param heading Starting heading
 */
void calibrate(degree_t heading);

/**
 * @brief Get the current x value
 *
 * @return double
 */
inch_t get_x();

/**
 * @brief Get the current y value
 *
 * @return double
 */
inch_t get_y();

}; // namespace odom