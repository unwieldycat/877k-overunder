#include "drive.hpp"
#include "devices.hpp"
#include "pros/misc.h"
#include <string>

#define WHEEL_OFFSET 7.6
int drive_mode = 1;

void drive::tank_drive() {
	int left_power = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
	int right_power = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
	drive_left.move(left_power);
	drive_right.move(right_power);
}

void drive::arcade_drive() {
	int left_y = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
	int right_x = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
	drive_left.move(left_y + right_x);
	drive_left.move(left_y + right_x);
}

void drive::curvature_drive() {
	int power = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
	float curvature =
	    controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X) / 127.0 * WHEEL_OFFSET;
	if (curvature != 0.0) {
		float radius = 1.0 / curvature;
		drive_left.move(power * (radius + WHEEL_OFFSET) / radius);
		drive_left.move(power * (radius - WHEEL_OFFSET) / radius);
	} else {
		drive_left.move(power);
		drive_right.move(power);
	}
}

void drive::drive() {
	while (true) {
		switch (drive_mode) {
		case 1:
			tank_drive();
			break;
		case 2:
			arcade_drive();
			break;
		case 3:
			curvature_drive();
			break;
		}

		pros::delay(50);
	}
}

void drive::select_drive_mode(int new_mode) { drive_mode = new_mode; }

void drive::select_drive_mode(std::string new_mode) {
	if (new_mode.compare("tank") == 0) {
		drive_mode = 1;
	}
	if (new_mode.compare("arcade") == 0) {
		drive_mode = 2;
	}
	if (new_mode.compare("curvature") == 0) {
		drive_mode = 3;
	}
}

int drive::get_drive_mode() { return drive_mode; }

std::string drive::get_drive_name() {
	switch (drive_mode) {
	case 1:
		return "tank";
		break;
	case 2:
		return "arcade";
		break;
	case 3:
		return "curvature";
		break;
	}
	return "no drive mode";
}