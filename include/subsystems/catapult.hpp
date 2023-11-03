#pragma once
#include "main.h"

namespace cata {

/**
 * @brief Checks if the catapult is primed
 * @returns Catapult state
 */
bool is_primed();

/**
 * @brief Prime the catapult
 */
void prime();

/**
 * @brief Release the catapult
 */
void release();

} // namespace cata