#include "main.h"

// TODO: Experiment with going forwards to clear triball
void awp_left() {
	odom::calibrate(11.25_ft, 1.5_ft, -45_deg);

	// Remove triball from matchload zone
	chassis::drive(1_ft);
	left_wing.extend();
	chassis::drive(-1_ft);
	chassis::turn_rel(-270_deg);

	// Drive to touch climbing bar
	left_wing.retract();
	chassis::drive(2.28_ft);
	chassis::turn_abs(90_deg); // FIXME: This turns 90 degrees, not to 90 degrees??
	chassis::drive(2_ft);
	right_wing.extend();
	chassis::turn_rel(-15_deg);
}

void awp_right() {
	odom::calibrate(9_ft, 11_ft, 45_deg);

	// Drive to side of goal
	chassis::drive(2.4_ft, 45_deg);
	chassis::turn_abs(0_deg);

	// Ram triball into goal
	chassis::drive(127, 0_deg, 1_s);
	chassis::drive(-104, 0_deg, 750_ms);

	// Drive to center
	chassis::drive(8_in, 0_deg);
	chassis::turn_abs(-90_deg);
	chassis::drive(6_ft, -90_deg);

	// Touch bar
	left_wing.extend();
	chassis::drive(3_in, 0_deg);
	chassis::turn_rel(15_deg);
}
