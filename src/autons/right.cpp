#include "devices.hpp"
#include "main.h"

void right() {
	odom::calibrate(0_ft, 0_ft, 0_deg);

	// Align to gather triballs
	chassis::drive(2_ft, 0_deg);
	chassis::turn_abs(-45_deg);

	left_wing.extend();
	chassis::drive(1.5_ft);
	chassis::turn_abs(0_deg);

	// Turn to face goal
	chassis::drive(0.25_ft);
	chassis::turn_abs(90_deg, chassis::TurnSide::Left);

	chassis::drive(127, 90_deg, 2_s);
	chassis::drive(-2_ft, 90_deg);
}