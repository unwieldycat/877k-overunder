#include "main.h"

void right() {
	odom::calibrate(0_ft, 0_ft, 0_deg);

	// Align to gather triballs
	chassis::drive(2_ft, 0_deg);
	chassis::turn_abs(-45_deg);
	left_wing.extend();
	chassis::drive(2.82_ft);
	chassis::turn_abs(0_deg);

	// Turn to face goal
	chassis::drive(1_ft, 0_deg);
	chassis::turn_abs(90_deg);

	// Push triballs into goal
	chassis::drive(127, 90_deg, 2_s);
	chassis::drive(-2_ft, 90_deg);
	chassis::drive(127, 90_deg, 2_s);
	chassis::drive(-1_ft, 90_deg);
}