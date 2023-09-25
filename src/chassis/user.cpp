#include "chassis.hpp"
#include "devices.hpp"
#include "odom.hpp"
#include "pid.hpp"

// ========================= User Control Functions ========================= //

chassis::ControlMode drive_mode = chassis::ControlMode::ARCADE;

void tank_drive() {
	int left_power = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
	int right_power = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
	drive_left.move(left_power);
	drive_right.move(right_power);
}

void arcade_drive() {
	int left_y = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
	int right_x = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
	drive_left.move(left_y + right_x);
	drive_right.move(left_y - right_x);
}

void curvature_drive() {
	double power = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) / 127.0;
	double curvature = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X) / 127.0;

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

void chassis::user_control() {
	while (true) {
		switch (drive_mode) {
		case ControlMode::TANK:
			tank_drive();
			break;
		case ControlMode::ARCADE:
			arcade_drive();
			break;
		case ControlMode::CURVE:
			curvature_drive();
			break;
		}

		if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
			roller_piston.toggle();
		}
		pros::delay(50);
	}
}

void chassis::set_mode(chassis::ControlMode new_mode) { drive_mode = new_mode; }

chassis::ControlMode chassis::get_mode() { return drive_mode; }
