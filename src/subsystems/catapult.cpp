#include "subsystems/catapult.hpp"
#include "devices.hpp"
#include "pros/misc.h"
#include "pros/rtos.hpp"

bool cata::is_primed() { return cata_rot.get_position() < 40; }

void cata::prime() {
	int max_time = pros::millis() + 8000;
	while (cata_rot.get_position() < 4000 && pros::millis() < max_time) {
		catapult.move(-127);
		pros::delay(10);
	}
}

void cata::release() {
	if (cata_rot.get_position() < 40) return; // Make sure cata is primed

	catapult.move(-64);
	while (cata_rot.get_position() > 40)
		pros::delay(50);
	catapult.move(0);
}

void cata::user() {
	while (true) {
		if (!controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
			pros::delay(20);
			return;
		}

		if (!cata::is_primed()) cata::prime();
		cata::release();
		cata::prime();
		pros::delay(20);
	}
}