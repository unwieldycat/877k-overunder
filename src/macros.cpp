#include "macros.hpp"
#include "chassis/auton.hpp"
#include "devices.hpp"
#include "input.hpp"
#include "subsystems/catapult.hpp"

void macros::reverse() {
	bool reversed = input::get_drive_reverse();
	input::set_drive_toggle(false);
	chassis::turn_rel(180_deg);
	input::set_drive_reverse(!reversed);
	input::set_drive_toggle(true);
}

void macros::catapult() {
	if (!cata::is_primed()) cata::prime();
	cata::release();
	cata::prime();
}