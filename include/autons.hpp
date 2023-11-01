#pragma once
#include "main.h"

/**
 * Park robot next to left goal, starting from left side
 */
void park_left();

/**
 * Park robot next to right goal, starting from right side
 */
void park_right();

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