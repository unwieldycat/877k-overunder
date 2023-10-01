#pragma once
#include "main.h"

// ============================== Input Manager ============================== //

namespace input {

typedef std::pair<pros::controller_digital_e_t, std::function<void()>> button_pair_t;

/**
 * @brief Set button mappings
 *
 * @param map Button to function map
 */
void set_buttons(std::vector<button_pair_t> map);

/**
 * @brief Task function to watch for button actions
 */
[[noreturn]] void watcher();

} // namespace input
