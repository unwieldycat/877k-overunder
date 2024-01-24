#include "main.h"

const double boost_pitch = 10;

void booster::automatic() {
	bool extended = false;
	while (true) {
		double pitch = imu.get_pitch();

		if (fabs(pitch) < boost_pitch && extended) {
			left_lift.retract();
			right_lift.retract();
			extended = false;
		}

		if (fabs(pitch) > boost_pitch && !extended) {
			left_lift.extend();
			right_lift.extend();
			extended = true;
		}

		pros::delay(100);
	}
}