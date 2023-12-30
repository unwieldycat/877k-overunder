#pragma once
#include "main.h"

namespace chassis {

/**
 * Drive with motor power
 */
void drive(int power);

/**
 * Drive a distance
 */
void drive(foot_t distance);

/**
 * Drive a distance with a specified heading
 */
void drive(foot_t distance, degree_t heading);

/**
 * Turn to a specific heading
 */
void turn_abs(degree_t heading);

/**
 * Turn relative to the robot's current position
 */
void turn_rel(degree_t degrees);

} // namespace chassis