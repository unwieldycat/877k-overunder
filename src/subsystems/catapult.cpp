#include "subsystems/catapult.hpp"
#include "devices.hpp"

bool cata::is_primed() { return cata_rot.get_position() <= 10; }

void cata::prime() {
	while (cata_rot.get_position() > 10) {
		catapult.move(64);
		pros::delay(10);
	}
}

void cata::release() {
	if (cata_rot.get_position() > 10) return; // Make sure cata is primed

	catapult.move(16);
	while (cata_rot.get_position() > 0)
		pros::delay(50);
	catapult.move(0);
}