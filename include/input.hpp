#pragma once
#include "main.h"

// ============================== Input Manager ============================== //

namespace input {

typedef std::pair<pros::controller_digital_e_t, std::function<void()>> button_pair_t;

typedef struct stick_values {
	int32_t x;
	int32_t y;
} stick_values_t;

typedef struct analog_inputs {
	stick_values_t left;
	stick_values_t right;
} analog_inputs_t;

/**
 * @brief Set the drive function
 *
 * @param drive_func Drive function
 */
void set_drive(std::function<void(analog_inputs_t)> drive_func);

/**
 * @brief Set button mappings
 *
 * @param map Button to function map
 */
void set_buttons(std::vector<button_pair_t> map);

/**
 * @brief Set stick deadzone
 *
 * @param deadzone Highest value to ignore from analog stick
 */
void set_deadzone(int deadzone);

/**
 * @brief Toggle the drive control
 *
 * @param state
 */
void set_drive_toggle(bool state);

/**
 * @brief Set drive control direction
 *
 * @param state True if reversed
 */
void set_drive_reverse(bool state);

/**
 * @brief Task function to watch for button actions
 */
[[noreturn]] void watcher();

/**
 * @brief Task function for driver control
 */
[[noreturn]] void driver();

} // namespace input
