#pragma once

namespace puncher {

/**
 * @brief Checks if the puncher is primed
 * @returns Puncher state
 */
bool is_primed();

/**
 * @brief Prime the puncher
 */
void prime();

/**
 * @brief Release the puncher
 */
void release();

/**
 * @brief User function for puncher
 */
void user();

} // namespace puncher