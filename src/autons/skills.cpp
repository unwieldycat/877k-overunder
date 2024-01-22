#include "main.h"

const int launch_time = 45000;

void skills() {
	odom::calibrate(2_ft, 2_ft, 225_deg); // TODO: Find correct angle?

	// Run kicker for specified time
	punch_motors.move(104);

	int start_time = pros::millis();
	while (start_time - pros::millis() < launch_time)
		pros::delay(100);

	punch_motors.brake();

	// Realign robot
	chassis::drive(64, 1_s);
	chassis::turn_abs(0_deg);
	chassis::drive(127, 2_s);
	odom::calibrate(3_ft, 11_ft + 3_in, 0_deg);
	chassis::drive(3_in, 0_deg);
	chassis::turn_abs(90_deg);

	// Drive to other side of field
	chassis::drive(6_ft, 90_deg);
	chassis::turn_abs(0_deg);
	chassis::drive(2_ft);
	chassis::turn_abs(-45_deg);
	chassis::drive(1.41_ft, -45_deg);

	// Push triballs
	left_wing.extend();
	right_wing.extend();
	chassis::turn_abs(0_deg);
	chassis::drive(96, 0_deg, 1_s);
	chassis::turn_abs(90_deg);

	// Push into goal
	chassis::drive(-96, 90_deg, 1_s);
	chassis::drive(127, 90_deg, 1.5_s);
	chassis::drive(-96, 90_deg, 1_s);

	// chassis::pursuit("/usd/paths/skills0.csv", true);
	// chassis::pursuit("/usd/paths/skills1.csv");
}
