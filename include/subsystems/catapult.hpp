#pragma once

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

/**
 * @brief User function for catapult
 */
void user();

} // namespace cata