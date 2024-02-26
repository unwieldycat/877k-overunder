#include "devices.hpp"
#include "main.h"
#include "subsystems/puncher.hpp"

void push_left() {
	chassis::d_drive(20, 50);
	pros::delay(300);
	chassis::d_turn(-70.0, 50);
	pros::delay(300);
	chassis::d_drive(2, 50);

	pros::delay(1000);

	chassis::d_drive(-4, 50);
	pros::delay(300);
	chassis::d_turn(90);
	pros::delay(300);
	chassis::drive(127, 1.1_s);
	pros::delay(300);

	// Cross
	chassis::drive(-127, 750_ms);
	pros::delay(300);

	/*


	odom::calibrate(2_ft, 11_ft, -45_deg);

	// Ram triball into goal
	chassis::drive(127, 0_deg, 1_s);
	chassis::drive(-104, 0_deg, 500_ms);*/
}

void push_right() {
	odom::calibrate(0_ft, 0_ft, 45_deg);
	puncher::hold();
	chassis::drive(2_ft, 45_deg);
	chassis::turn_abs(0_deg);
	puncher::unhold();
	chassis::drive(127, 1_s);
	chassis::drive(-64, 750_ms);
	pros::delay(1000);
	puncher::hold();
	chassis::drive(127, 0_deg, 1_s);
	chassis::drive(-64, 0_deg, 750_ms);
	puncher::unhold();

	/* Temp auton
	chassis::d_drive(20, 50);
	pros::delay(300);
	chassis::d_turn(70.0, 50);
	pros::delay(300);
	chassis::d_drive(2, 50);

	chassis::d_drive(-4, 50);
	chassis::d_turn(-90);
	chassis::drive(127, 1.1_s);
	chassis::drive(-127, 750_ms);*/
}
