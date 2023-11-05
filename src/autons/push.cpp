#include "autons.hpp"
#include "chassis/auton.hpp"
#include "devices.hpp"

void push_field() {
	chassis::drive(4_ft);
	chassis::turn_rel(-90_deg);
	chassis::drive(2.82_ft); // 2^2 + 2^2 = c^2
	chassis::turn_rel(-90_deg);
	chassis::drive(1_ft);
}

void push_preload() {
	chassis::drive(2.82_ft);
	chassis::turn_rel(-90_deg);
	chassis::drive(1_ft);
}

void drive_forward() {
	drive_left.move(127);
	drive_right.move(127);
}