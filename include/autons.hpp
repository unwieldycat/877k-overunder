#pragma once

/**
 * Push the field triball into goal
 */
void push_field();

/**
 * Push a preload triball into goal
 */
void push_preload();

/**
 * Skills run. Parks robot at matchloader, then pushes triballs in goal.
 * Starts on second tile from blue low bar
 */
void skills();

/**
 * Does half of the autonomous win point. Scores triball and touches climb bar
 * Starts on first tile from blue low bar facing midline.
 */
void awp();