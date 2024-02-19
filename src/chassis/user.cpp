#include "main.h"

// ========================= User Control Functions ========================= //

void tank_drive(int left_x, int left_y, int right_x, int right_y) {
	int left = left_y;
	int right = right_y;
	drive_left.move(left);
	drive_right.move(right);
}

void arcade_drive(int left_x, int left_y, int right_x, int right_y) {
	int left = left_y + right_x;
	int right = left_y - right_x;
	drive_left.move(left);
	drive_right.move(right);
}

void curvature_drive(int left_x, int left_y, int right_x, int right_y) {
	double power = left_y / 127.0;
	double curvature = right_x / 127.0;

	double left = power + power * curvature;
	double right = power - power * curvature;
	double max = std::max(std::fabs(left), std::fabs(right));

	if (max > 1.0) {
		left /= max;
		right /= max;
	}

	drive_left.move(left * 127.0);
	drive_right.move(right * 127.0);
}

// ========================= Input Processor & Task ========================= //

const int deadzone = 1;
bool enable_drive = true;
bool reverse_drive = false;
std::function<void(int, int, int, int)> driver_func = arcade_drive;

int process_input(int value) {
	if (abs(value) < deadzone) return 0;
	int rescaled = ((double)value - deadzone) / (127 - deadzone) * 127;
	return (reverse_drive) ? -rescaled : rescaled;
}

void chassis::reverse() {
	reverse_drive = !reverse_drive;
	enable_drive = false;
	chassis::turn_rel(180_deg);
	enable_drive = true;
}

void chassis::set_drive_mode(chassis::drive_mode_t mode) {
	if (mode == DRIVE_MODE_ARCADE)
		driver_func = arcade_drive;
	else if (mode == DRIVE_MODE_CURVE)
		driver_func = curvature_drive;
	else if (mode == DRIVE_MODE_TANK)
		driver_func = tank_drive;
}

void chassis::user() {
	while (true) {
		if (!enable_drive) {
			pros::delay(10);
			continue;
		};

		int left_x = process_input(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X));
		int left_y = process_input(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y));
		int right_x = process_input(controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X));
		int right_y = process_input(controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y));

		driver_func(left_x, left_y, right_x, right_y);
		pros::delay(10);
	}
}