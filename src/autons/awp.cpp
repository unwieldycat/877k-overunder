#include "main.h"

void awp_left() {
	// TODO: Make it do stuff
	// Run push auton
	// Go to climbing bar and tap somehow
	odom::calibrate(11_ft, 3_ft, 180_deg);
	chassis::pursuit("/usd/paths/awp_auton0.csv");
	chassis::pursuit("/usd/paths/awp_auton1p.csv", true);
}

void awp_right() {
	chassis::drive(2.5_ft);
	right_wing.extend();
	chassis::turn_abs(-45_deg);
	chassis::drive(2_ft);
	left_wing.extend();
	chassis::drive(12_in, 90_deg);
	chassis::drive(127);
	pros::delay(1000);
	chassis::drive(-127);
	pros::delay(1000);
	chassis::drive(0);
	pros::delay(2000);
	chassis::drive(3_in, 90_deg);
	right_wing.retract();
	left_wing.retract();
	chassis::turn_abs(180_deg);
	chassis::drive(2.5_ft, 180_deg);
	right_wing.extend();
}
