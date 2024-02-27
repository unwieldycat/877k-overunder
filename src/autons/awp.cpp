#include "main.h"

void awp() {
	odom::calibrate(11.25_ft, 1.5_ft, -45_deg);

	// Remove triball
	left_wing.extend();
	pros::delay(1000);
	chassis::turn_rel(-37_deg, chassis::TurnSide::Right);
	pros::delay(2000);

	// Drive to touch climbing bar
	chassis::turn_abs(-135_deg);
	left_wing.retract();
	chassis::drive(-1.9_ft);
	chassis::turn_abs(90_deg);
	chassis::drive(64, 92_deg, 1.5_s);

	// Touch climbing bar
	right_wing.extend();
	pros::delay(1000);
	chassis::turn_rel(-15_deg);
}
