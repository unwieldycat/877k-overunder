#pragma once

namespace puncher {

/**
 * @brief Checks if the puncher is primed
 * @returns Puncher state
 */
bool is_primed();

/**
 * @brief Check if the puncher is holding
 *
 * @return Puncher state
 */
bool is_hold();

/**
 * @brief Prime the puncher
 */
void prime();

/**
 * @brief Hold triball
 */
void hold();

/**
 * @brief Release the puncher
 */
void release();

/**
 * @brief Unhold puncher
 */
void unhold();

/**
 * @brief User function for puncher
 */
void user();

} // namespace puncher