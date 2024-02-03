#include "main.h"

const int launch_time = 45000;

void skills() {
	// Align for shot

	chassis::d_drive(20, 50);
	pros::delay(300);
	chassis::d_turn(-70.0, 50);
	pros::delay(300);
	chassis::d_drive(2, 50);

	// odom::calibrate(2_ft, 2_ft, -120_deg);

	// Run kicker for specified time
	left_wing.extend();
	punch_motors.move(104);

	int start_time = pros::millis();
	while (pros::millis() - start_time < launch_time)
		pros::delay(100);

	punch_motors.brake();
	pros::delay(300);
	left_wing.retract();

	// Push
	chassis::d_drive(-4, 50);
	pros::delay(300);
	chassis::d_turn(90);
	pros::delay(300);
	chassis::d_drive(10, 127);
	pros::delay(300);
	chassis::d_drive(-10);
	pros::delay(300);
	chassis::d_drive(10, 127);
	pros::delay(300);

	// Cross
	chassis::d_drive(-10);
	pros::delay(300);

	return;

	chassis::d_turn(120);
	pros::delay(300);
	chassis::d_drive(16);
	pros::delay(300);
	chassis::d_turn(-20);
	pros::delay(300);
	/*chassis::d_turn(-15);
	pros::delay(300);*/
	chassis::d_drive(64, 80);
	pros::delay(300);

	// Push
	chassis::d_turn(-90, 55);
	pros::delay(300);
	chassis::d_drive(24);
	pros::delay(300);
	chassis::d_turn(-20, 55);
	pros::delay(300);
	chassis::d_drive(32);
	pros::delay(300);
	chassis::d_turn(110, 55);
	left_wing.extend();
	right_wing.extend();
	pros::delay(300);
	chassis::d_drive(24);
	pros::delay(300);
	chassis::d_drive(-24);
	pros::delay(300);
	chassis::d_drive(24);
	pros::delay(300);
	chassis::d_drive(-24);
	pros::delay(300);

	/*
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
	// chassis::pursuit("/usd/paths/skills1.csv");*/
}
