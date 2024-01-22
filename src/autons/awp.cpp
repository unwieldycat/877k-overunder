#include "main.h"

void awp_left() {
	odom::calibrate(11.25_ft, 1.5_ft, -45_deg);

	// Remove triball from matchload zone
	chassis::drive(1_ft);
	left_wing.extend();
	chassis::drive(-1_ft);
	chassis::turn_rel(-90_deg);

	// Drive to touch climbing bar
	left_wing.retract();
	chassis::drive(-2.28_ft);
	chassis::turn_abs(90_deg);
	chassis::drive(96, 3_s);
	right_wing.extend();
	chassis::turn_rel(-15_deg);
}

void awp_right() {
	odom::calibrate(9_ft, 11_ft, 45_deg);

	// Drive to side of goal
	chassis::drive(2.82_ft, 45_deg);
	chassis::turn_abs(0_deg);

	// Ram triball into goal
	chassis::drive(127, 0_deg, 1_s);
	chassis::drive(-104, 0_deg, 1_s);
	chassis::drive(127, 0_deg, 1_s);

	// Drive to center
	chassis::drive(-6_in);
	chassis::turn_abs(-90_deg);
	chassis::drive(6_ft, -90_deg);

	// Touch bar
	left_wing.extend();
	chassis::drive(96, 1_s);
	chassis::turn_rel(-15_deg);
}
