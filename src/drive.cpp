#include "drive.hpp"
#include "devices.hpp"
#include "pros/misc.h"
#include <string>

#define WHEEL_OFFSET 7.6
drive::DriveMode drive_mode;

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
		case DriveMode::TANK:
			tank_drive();
			break;
		case DriveMode::ARCADE:
			arcade_drive();
			break;
		case DriveMode::CURVE:
			curvature_drive();
			break;
		}

		pros::delay(50);
	}
}

void drive::set_mode(DriveMode new_mode) { drive_mode = new_mode; }

drive::DriveMode drive::get_mode() { return drive_mode; }
