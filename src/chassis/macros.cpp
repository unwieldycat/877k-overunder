#include "macros.hpp"
#include "chassis.hpp"
#include "input.hpp"

void reverse() {
	bool reversed = input::get_drive_reverse();
	input::set_drive_toggle(false);
	chassis::turn_rel(180_deg);
	input::set_drive_reverse(!reversed);
	input::set_drive_toggle(true);
}