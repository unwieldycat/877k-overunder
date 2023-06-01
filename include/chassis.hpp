#pragma once
#include "main.h"
#include "pid.hpp"

namespace chassis {

/**
 * Drive a distance (feet)
 */
void drive(double distance);

/**
 * Turn to a specific heading
 */
void turn_abs(double heading);

/**
 * Turn relative to the robot's current position
 */
void turn_rel(double degrees);

/**
 * User control mode
 */
enum class ControlMode { TANK, ARCADE, CURVE };

/**
 * User control function
 *
 * This function is blocking and should be ran inside a task
 */
void user_control();

/**
 * Set the user control mode
 */
void set_mode(ControlMode drive_mode);

/**
 * Get the user control mode
 */
ControlMode get_mode();

} // namespace chassis