#include "main.h"

void push_left() {
	odom::calibrate(0_ft, 0_ft, 45_deg);
	chassis::drive(2.2_ft);
	chassis::drive(127, 0_deg, 2_s);
	chassis::drive(-127, 0_deg, 1_s);
	chassis::drive(127, 0_deg, 1_s);
}

void push_right() {
	odom::calibrate(0_ft, 0_ft, -45_deg);
	chassis::drive(2.2_ft);
	chassis::drive(127, 0_deg, 2_s);
	chassis::drive(-127, 0_deg, 1_s);
	chassis::drive(127, 0_deg, 1_s);
}
