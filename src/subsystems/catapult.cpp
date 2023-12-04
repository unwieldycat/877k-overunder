#include "subsystems/catapult.hpp"
#include "devices.hpp"
#include "pros/motors.h"

const int release_angle = -4000;

bool cata::is_primed() { return cata_rot.get_position() < release_angle; }

void cata::prime() {
	while (cata_rot.get_position() > release_angle) {
		catapult.move(127);
		pros::delay(10);
	}
	catapult.brake();
}

void cata::release() {
	if (cata_rot.get_position() > release_angle) return; // Make sure cata is primed

	catapult.move(127);
	while (cata_rot.get_position() < release_angle)
		pros::delay(50);
	catapult.brake();
}

void cata::user() {
	bool continuous = false;
	bool brake = false;
	catapult.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	while (true) {
		if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)) continuous = !continuous;

		/* FIXME: Catapult never stops
		if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B) && !continuous) {
		    if (!cata::is_primed()) cata::prime();
		    cata::release();
		    cata::prime();
		    pros::delay(20);
		    return;
		}*/

		if (continuous) {
			catapult.move(96);
		} else {
			catapult.brake();
		}

		pros::delay(20);
	}
}