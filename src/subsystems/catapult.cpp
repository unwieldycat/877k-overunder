#include "subsystems/catapult.hpp"
#include "devices.hpp"

const int release_angle = 4000;

bool cata::is_primed() { return cata_rot.get_position() < release_angle; }

void cata::prime() {
	while (cata_rot.get_position() < release_angle) {
		catapult.move(127);
		pros::delay(10);
	}
}

void cata::release() {
	if (cata_rot.get_position() < release_angle) return; // Make sure cata is primed

	catapult.move(127);
	while (cata_rot.get_position() > release_angle)
		pros::delay(50);
	catapult.move(0);
}

void cata::user() {
	bool continuous = false;
	bool brake = false;
	while (true) {
		if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A))
			continuous = !continuous;

		if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B) && !continuous) {
			if (!cata::is_primed()) cata::prime();
			cata::release();
			cata::prime();
			pros::delay(20);
			return;
		}

		if (continuous) {
			catapult.move(127);
			pros::delay(20);
			return;
		}

		catapult.brake();
		pros::delay(20);
	}
}