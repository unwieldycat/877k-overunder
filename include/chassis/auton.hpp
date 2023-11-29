#pragma once

#include "main.h"
#include "pid.hpp"

namespace chassis {

/**
 * Drive a distance
 */
void drive(foot_t distance);

/**
 * Turn to a specific heading
 */
void turn_abs(degree_t heading);

/**
 * Turn relative to the robot's current position
 */
void turn_rel(degree_t degrees);

} // namespace chassis