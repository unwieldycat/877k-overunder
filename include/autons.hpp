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
 * Achieves the autonomous win point. Scores triball, removes triball, and touches climbing
 * bar.
 */
void awp();

/**
 * Higher-scoring auton for right side, pushes ~3 triballs into goal.
 */
void right();
