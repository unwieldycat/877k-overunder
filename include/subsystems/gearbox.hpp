#pragma once

namespace gearbox {
/**
 * @brief Shifts the gearbox into speed mode
 */
void shift_speed();

/**
 * @brief Shifts the gearbox into torque mode
 */
void shift_torque();

/**
 * @brief User control function
 */
void user();
} // namespace gearbox
