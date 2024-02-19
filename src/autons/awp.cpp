#include "main.h"

void awp() {
	odom::calibrate(11.25_ft, 1.5_ft, -45_deg);

	// FIXME: wing cannot remove triball
	// Remove triball
	chassis::drive(6_in);
	left_wing.extend();
	chassis::drive(-6_in);
	chassis::turn_rel(-45_deg, chassis::TurnSide::Right);

	// FIXME: Tune distances to align properly
	// Drive to touch climbing bar
	chassis::turn_abs(-135_deg);
	left_wing.retract();
	chassis::drive(-2_ft);
	chassis::turn_abs(90_deg);
	chassis::drive(2_ft, 90_deg);

	// Touch climbing bar
	right_wing.extend();
	chassis::turn_rel(-15_deg);
}
