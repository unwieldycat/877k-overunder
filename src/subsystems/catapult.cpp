#include "subsystems/catapult.hpp"
#include "devices.hpp"
#include "pros/motors.h"

const int release_angle = 7500;

bool cata::is_primed() { return cata_rot.get_position() >= release_angle; }

void cata::prime() {
	while (cata_rot.get_position() < release_angle) {
		catapult.move(127);
		pros::delay(10);
	}
	catapult.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	catapult.brake();
}

void cata::release() {
	if (!cata::is_primed()) return; // Make sure cata is primed

	catapult.move(127);
	while (cata_rot.get_position() >= release_angle)
		pros::delay(50);

	catapult.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	catapult.brake();
}

void cata::user() {
	bool continuous = false;
	bool brake = false;
	cata_rot.reset_position();
	while (true) {
		if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A))
			continuous = !continuous;

		if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B) && !continuous) {
			std::cout << cata_rot.get_position() << "\n";
			std::cout << cata::is_primed() << "\n";
			if (!cata::is_primed())
				cata::prime();
			else {
				cata::release();
				pros::delay(250);
				cata::prime();
			}

			pros::delay(20);
			continue;
		} else if (continuous) {
			catapult.move(104);
		} else {
			catapult.brake();
		}

		pros::delay(20);
	}
}