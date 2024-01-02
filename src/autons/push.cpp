#include "main.h"

void push_field() {
	chassis::drive(4_ft);
	chassis::turn_rel(-90_deg);
	chassis::drive(2.82_ft); // 2^2 + 2^2 = c^2
	chassis::turn_rel(-90_deg);
	chassis::drive(1_ft);
}

void push_preload() {
	chassis::drive(4_ft);
	left_wing.extend();
	chassis::turn_abs(90_deg);
	chassis::drive(127);
	pros::delay(750);
	chassis::drive(-127);
	pros::delay(750);
	chassis::drive(0);
	left_wing.retract();
}
