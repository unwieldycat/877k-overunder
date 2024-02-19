#include "main.h"

const double boost_pitch = -15;
const int boost_time = 2000;

void booster::automatic() {
	bool extended = false;
	int start_pitch = 0;

	while (true) {
		double pitch = imu.get_roll();
		int pitch_time = pros::millis() - start_pitch;

		if (pitch > boost_pitch && extended && pitch_time > boost_time) {
			left_lift.retract();
			right_lift.retract();
			extended = false;
		}

		if (pitch < boost_pitch && !extended) {
			left_lift.extend();
			right_lift.extend();
			extended = true;
			start_pitch = pros::millis();
		}

		pros::delay(100);
	}
}