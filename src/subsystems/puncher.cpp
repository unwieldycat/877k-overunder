#include "main.h"

const int release_angle = -7500; // 7500

bool puncher::is_primed() { return puncher_rot.get_position() <= release_angle; }

void puncher::prime() {
	while (puncher_rot.get_position() > release_angle) {
		catapult.move(127);
		pros::delay(10);
	}
	catapult.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	catapult.brake();
}

void puncher::release() {
	if (!puncher::is_primed()) return; // Make sure cata is primed

	catapult.move(127);
	while (puncher_rot.get_position() <= release_angle)
		pros::delay(50);

	catapult.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	catapult.brake();
}

void puncher::user() {
	bool continuous = false;
	bool brake = false;
	puncher_rot.reset_position();
	while (true) {
		if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A))
			continuous = !continuous;

		if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B) && !continuous) {
			if (!puncher::is_primed())
				puncher::prime();
			else {
				puncher::release();
				pros::delay(250);
				puncher::prime();
			}

			pros::delay(20);
			continue;
		} else if (continuous) {
			catapult.move(104);

		} else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
			catapult.move(-127);
			pros::delay(500);
			catapult.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
			catapult.brake();
		} else {
			catapult.brake();
		}

		pros::delay(20);
	}
}