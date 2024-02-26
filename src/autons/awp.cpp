#include "main.h"

void awp() {
	odom::calibrate(11.25_ft, 1.5_ft, -45_deg);

	// FIXME: wing cannot remove triball
	// Remove triball
	// chassis::drive(1_ft);
	left_wing.extend();
	// chassis::drive(-6_in);
	chassis::turn_rel(-37_deg, chassis::TurnSide::Right);
	pros::delay(2000);

	// FIXME: Tune distances to align properly
	// Drive to touch climbing bar
	chassis::turn_abs(-135_deg);
	left_wing.retract();
	chassis::drive(-1.9_ft);
	chassis::turn_abs(90_deg);
	chassis::drive(1.8_ft, 92_deg);

	// Touch climbing bar
	right_wing.extend();
	pros::delay(1000);
	chassis::turn_rel(-15_deg);
}
