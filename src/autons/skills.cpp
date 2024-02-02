#include "main.h"

const int launch_time = 45000;

void skills() {
	odom::calibrate(2_ft, 2_ft, -120_deg);

	// Run kicker for specified time
	punch_motors.move(104);

	int start_time = pros::millis();
	while (start_time - pros::millis() < launch_time)
		pros::delay(100);

	punch_motors.brake();

	// Recalibrate
	drive_right.move(96);
	pros::delay(1000);
	drive_right.brake();
	odom::calibrate(2_ft, 11.5_ft, -135_deg);

	chassis::drive(-64, 250_ms);

	// Align robot to cross field
	chassis::turn_rel(-135_deg);
	chassis::turn_rel(30_deg);
	chassis::drive(9_in);
	chassis::turn_rel(-30_deg);

	// Drive to other side of field
	chassis::drive(4_ft);
	chassis::drive(1.4_ft);
	chassis::turn_rel(-90_deg);

	chassis::drive(2_ft);
	chassis::turn_rel(-45_deg);
	chassis::drive(1.41_ft);

	// Push triballs
	left_wing.extend();
	right_wing.extend();

	chassis::turn_rel(45_deg);
	chassis::drive(96, 0_deg, 1_s);
	chassis::turn_rel(90_deg);

	// Push into goal
	chassis::drive(96, 90_deg, 1_s);
	chassis::drive(-96, 90_deg, 1_s);
	chassis::drive(127, 90_deg, 1_s);
	chassis::drive(-96, 90_deg, 1_s);

	// chassis::pursuit("/usd/paths/skills0.csv", true);
	// chassis::pursuit("/usd/paths/skills1.csv");
}
