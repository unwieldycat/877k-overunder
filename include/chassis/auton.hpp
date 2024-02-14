#pragma once
#include "main.h"

namespace chassis {

/**
 * Drive with motor power
 */
void drive(int power, millisecond_t time);

/**
 * Drive with motor power at a heading
 */
void drive(int power, degree_t heading, millisecond_t time);

/**
 * Drive a distance
 */
void drive(foot_t distance);

/**
 * Drive a distance with a specified heading
 */
void drive(foot_t distance, degree_t heading);

/**
 * Which wheels to turn when turning
 */
enum class TurnSide { Left, Right, Both };

/**
 * Turn to a specific heading
 */
void turn_abs(degree_t heading, TurnSide side = TurnSide::Both);

/**
 * Turn relative to the robot's current position
 */
void turn_rel(degree_t degrees, TurnSide side = TurnSide::Both);

void d_drive(double dist, int power = 70);

void d_turn(double angle, int power = 70);

} // namespace chassis