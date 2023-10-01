#include "input.hpp"
#include "devices.hpp"

// ============================== Input Manager ============================== //

std::vector<input::button_pair_t> button_map;

void set_buttons(std::vector<input::button_pair_t> map) { button_map = map; }

[[noreturn]] void input::watcher() {
	while (true) {
		for (const input::button_pair_t pair : button_map) {
			if (!controller.get_digital_new_press(pair.first)) continue;
			pair.second();
		}

		pros::delay(10);
	}
};