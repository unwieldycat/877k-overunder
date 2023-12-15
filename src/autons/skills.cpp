#include "autons.hpp"
#include "chassis/auton.hpp"
#include "chassis/pursuit.hpp"
#include "devices.hpp"
#include "odom.hpp"
#include "subsystems/catapult.hpp"

const int launches = 50;

void skills() {
	// Drive to match loader
	chassis::drive(2_ft);
	chassis::turn_rel(-135_deg);
	chassis::drive(2_ft);
	chassis::drive(64);

	// Match load
	// TODO: Display countdown on screen & change LED colors

	cata_optical.set_led_pwm(100);
	int end_time = pros::millis() + 30000;

	for (int i = 0; i < launches; i++) {
		cata::prime();
		cata::release();
		pros::delay(20);
	}

	// Prepare for pursuit
	chassis::drive(1_ft);
	chassis::turn_rel(-45_deg);
	chassis::drive(1.41_ft);
	chassis::turn_rel(45_deg);

	// TODO: Pursuit
}

void cata_only() {
	odom::calibrate(1_ft, 3_ft, 90_deg);
	chassis::pursuit::add_point(1_ft, 3_ft, 0.24845);
	chassis::pursuit::add_point(1.5625_ft, 2.3125_ft, 0.08615);
	chassis::pursuit::add_point(1.75_ft, 1.75_ft, 0.09843);
	chassis::pursuit::add_point(1.5625_ft, 1.3125_ft, 0.08362);
	chassis::pursuit::add_point(1_ft, 1_ft, 0.04141);
	chassis::pursuit::pursuit();

	// TODO: Cata stuff here
}