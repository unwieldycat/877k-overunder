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
	chassis::drive(1.414_ft);
	left_wing.extend();
	chassis::turn_abs(0_deg);
	chassis::drive(1.5_ft, 0_deg);
	chassis::drive(1_ft, 90_deg);
	chassis::drive(127, 90_deg, 1_s);
	chassis::drive(-127, 90_deg, 2_s);
	chassis::drive(6_in, 90_deg);
	right_wing.retract();
	left_wing.retract();
	chassis::turn_abs(180_deg);
	chassis::drive(2.5_ft, 180_deg);
	right_wing.extend();
	chassis::turn_rel(-45_deg);
}
