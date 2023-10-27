#include "input.hpp"
#include "chassis/user.hpp"
#include "devices.hpp"

std::vector<input::button_pair_t> button_map;
std::function<void(input::analog_inputs_t)> driver_func = chassis::arcade_drive;
int deadzone = 1;
bool enable_drive = true;
bool reverse_drive = false;

int process_input(int value) {
	if (abs(value) < deadzone) return 0;
	int rescaled = ((double)value - deadzone) / (127 - deadzone) * 127;
	return (reverse_drive) ? -rescaled : rescaled;
}

// ============================ Setter Functions ============================ //

void input::set_deadzone(int deadzone) { deadzone = deadzone; }

void input::set_buttons(std::vector<input::button_pair_t> map) { button_map = map; }

void input::set_drive(std::function<void(input::analog_inputs_t)> drive_func) {
	driver_func = drive_func;
}

void input::set_drive_toggle(bool state) { enable_drive = state; }

void input::set_drive_reverse(bool state) { reverse_drive = state; }

bool input::get_drive_reverse() { return reverse_drive; }

// ============================= Task Functions ============================= //

[[noreturn]] void input::watcher() {
	while (true) {
		for (const input::button_pair_t pair : button_map) {
			if (!controller.get_digital_new_press(pair.first)) continue;
			pair.second();
		}

		pros::delay(10);
	}
};

[[noreturn]] void input::driver() {
	while (true) {
		if (!enable_drive) {
			pros::delay(10);
			continue;
		};

		input::analog_inputs_t inputs;
		inputs.left.x = process_input(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X));
		inputs.left.y = process_input(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y));
		inputs.right.x = process_input(controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X));
		inputs.right.y = process_input(controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y));

		driver_func(inputs);
		pros::delay(10);
	}
}