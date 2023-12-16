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
	chassis::drive(45);
	pros::delay(5000);
	chassis::drive(0);
	chassis::turn_rel(90_deg);
	left_wing.extend();
	chassis::drive(127);
	pros::delay(750);
	chassis::drive(-127);
	pros::delay(750);
	chassis::drive(0);
	left_wing.retract();
}
