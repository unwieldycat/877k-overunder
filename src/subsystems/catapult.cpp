#include "subsystems/catapult.hpp"
#include "devices.hpp"

void cata::prime() {
	while (!cata_switch.get_value()) {
		catapult.move(64);
		pros::delay(10);
	}
}

void cata::release() {
	if (!cata_switch.get_value()) return; // Make sure cata is primed

	catapult.move(16);
	while (cata_switch.get_value())
		pros::delay(50);
	catapult.move(0);
}