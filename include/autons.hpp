#pragma once

/**
 * Push the field triball into goal
 */
void push_right();

/**
 * Push a preload triball into goal
 */
void push_left();

/**
 * Skills run. Parks robot at matchloader, then pushes triballs in goal.
 * Starts on second tile from blue low bar
 */
void skills();

/**
 * Does left half of the autonomous win point. Scores triball, removes triball, and touches climbing
 * bar.
 */
void awp_left();

/**
 * Does right portion of autonomous win point. Scores triballs and touches cimbing bar.
 */
void awp_right();