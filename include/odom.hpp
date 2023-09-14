#pragma once
#include "main.h"

namespace odom {

/**
 * @brief Initialize odometry
 */
void initialize();

/**
 * @brief Convert local coordinates to global coordinates
 * 
 * @param local_x Local coordinate X
 * @param local_y Local coordinate Y
 * @param robot_heading Current heading
 * @return Global coordinate pair
 */
std::pair<double, double> local_to_global_coords(inch_t local_x, inch_t local_y, degree_t robot_heading);

/**
 * @brief Convert global coordinates to local coordinates
 * 
 * @param global_x Global coordinate X
 * @param global_y Global coordinate Y
 * @param robot_x Robot X position
 * @param robot_y Robot Y position
 * @param robot_heading Current heading
 * @return Local coordinate pair
 */
std::pair<double, double> global_to_local_coords(
    inch_t global_x, inch_t global_y, inch_t robot_x, inch_t robot_y, degree_t robot_heading
);

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
 * @brief Get the current x value
 * 
 * @return double 
 */
double get_x();

/**
 * @brief Get the current y value
 * 
 * @return double 
 */
double get_y();

}; // namespace odom