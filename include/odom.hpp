#pragma once
#include "main.h"

namespace odom {

// FIXME: Do we need init and calib?
/**
 * @brief Initialize odometry
 */
void initialize();

// FIXME: Functions should use pairs
// FIXME: Should there be a coordinate class?
/**
 * @brief Convert local coordinates to global coordinates
 * 
 * @param local_x Local coordinate X
 * @param local_y Local coordinate Y
 * @param robot_heading Current heading
 * @return Global coordinate pair
 */
std::pair<double, double> local_to_global_coords(double local_x, double local_y, double robot_heading);

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
    double global_x, double global_y, double robot_x, double robot_y, double robot_heading
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
void calibrate(double robot_x, double robot_y, double heading);

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