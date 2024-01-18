#pragma once
#include "main.h"

/**
 * @brief Slew calculator
 *
 * @param new_value Desired value
 * @param current_value Current value
 * @param max_delta Maximum voltage change
 * @return New voltage
 */
int slew(int new_value, int current_value, int max_delta);

/**
 * @brief Clamp value to an absolute bound
 *
 * @param value Desired value
 * @param max Absolute max value
 * @return int
 */
int clamp(int value, int max);

/**
 * @brief Clamp value to an upper & lower bound
 *
 * @param value Desired value
 * @param max Maximum value
 * @param min Minimum value
 * @return int
 */
int clamp(int value, int max, int min);