#include "autons.hpp"
#include "chassis/auton.hpp"
#include "subsystems/catapult.hpp"

void skills() {
	// Drive to match loader
	chassis::drive(2_ft);
	chassis::turn_rel(-45_deg);
	chassis::drive(-2.82_ft);

	// Match load
	// TODO: Display countdown on screen & change LED colors

	cata_optical.set_led_pwm(100);
	int end_time = pros::millis() + 30000;

	while (pros::millis() < end_time) {
		cata::prime();
		while (cata_optical.get_rgb().green > 200 && pros::millis() < end_time)
			pros::delay(20);

		pros::delay(500);
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