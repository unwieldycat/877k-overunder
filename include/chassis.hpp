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

} // namespace chassis