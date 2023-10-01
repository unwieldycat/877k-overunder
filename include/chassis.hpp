#pragma once
#include "input.hpp"
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

// User control functions

void curvature_drive(input::analog_inputs_t inputs);

void tank_drive(input::analog_inputs_t inputs);

void arcade_drive(input::analog_inputs_t inputs);

} // namespace chassis