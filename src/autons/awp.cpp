#include "main.h"

void awp() {
	odom::calibrate(11.25_ft, 1.5_ft, 90_deg);

	chassis::drive(2_ft);
	chassis::turn_abs(-135_deg);
	chassis::drive(2_ft + 3_in);

	// Remove triball from matchload zone
	chassis::turn_rel(30_deg);
	left_wing.extend();
	chassis::turn_abs(90_deg, chassis::TurnSide::Left);
	left_wing.retract();

	// TODO: Make triball removal consistent and find position robot ends

	// Realign to go to climber
	chassis::drive(-2_in);
	chassis::turn_abs(90_deg);

	// Drive to touch climbing bar
	chassis::drive(4_ft, 90_deg);
	right_wing.extend();
	chassis::turn_rel(-15_deg);
}
