#include "main.h"

void push_left() {
	odom::calibrate(2_ft, 11_ft, -45_deg);

	// Drive to side of goal
	chassis::drive(2.82_ft, -45_deg);
	chassis::turn_abs(0_deg);

	// Ram triball into goal
	chassis::drive(127, 0_deg, 1_s);
	chassis::drive(-104, 0_deg, 1_s);
	chassis::drive(127, 0_deg, 1_s);
}

void push_right() {
	odom::calibrate(9_ft, 11_ft, 45_deg);

	// Drive to side of goal
	chassis::drive(2.82_ft, 45_deg);
	chassis::turn_abs(0_deg);

	// Ram triball into goal
	chassis::drive(127, 0_deg, 1_s);
	chassis::drive(-104, 0_deg, 1_s);
	chassis::drive(127, 0_deg, 1_s);
}
