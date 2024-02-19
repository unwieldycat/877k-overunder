#include "main.h"

const int launch_time = 45000;

void skills() {
	odom::calibrate(0_ft, 0_ft, -45_deg);

	// Push preload in
	chassis::drive(2_ft);
	chassis::turn_abs(0_deg, chassis::TurnSide::Left);
	chassis::drive(127, 1_s);
	chassis::drive(-1_ft, 0_deg);

	// Align to launch
	chassis::turn_abs(180_deg);
	chassis::drive(8_in);
	chassis::turn_abs(-90_deg - 15_deg);
	left_wing.extend();

	punch_motors.move(104);

	int start_time = pros::millis();
	while (start_time - pros::millis() < launch_time)
		pros::delay(1000);

	punch_motors.brake();

	// TODO: push into goal

	// chassis::pursuit("/usd/paths/skills0.csv", true);
	// chassis::pursuit("/usd/paths/skills1.csv");
}
