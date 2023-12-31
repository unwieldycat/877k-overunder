#include "main.h"

const int release_angle = -7500; // 7500

bool puncher::is_primed() { return puncher_rot.get_position() <= release_angle; }

void puncher::prime() {
	while (puncher_rot.get_position() > release_angle) {
		punch_motors.move(127);
		pros::delay(10);
	}
	punch_motors.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	punch_motors.brake();
}

void puncher::release() {
	if (!puncher::is_primed()) return; // Make sure cata is primed

	punch_motors.move(127);
	while (puncher_rot.get_position() <= release_angle)
		pros::delay(50);

	punch_motors.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	punch_motors.brake();
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
			punch_motors.move(104);

		} else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
			punch_motors.move(-127);
			pros::delay(500);
			punch_motors.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
			punch_motors.brake();
		} else {
			punch_motors.brake();
		}

		pros::delay(20);
	}
}