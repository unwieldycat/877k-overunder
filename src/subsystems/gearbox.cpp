#include "main.h"

using namespace units::math;

void gearbox::shift_speed() { transmission.retract(); }

void gearbox::shift_torque() { transmission.extend(); }

// TODO: Measure shifting velocities (in/sec)
const double shift_up = 999.0;
const double shift_down = 0.0;

void gearbox::user() {
	bool automatic = true;
	int prev_time = pros::millis();
	inch_t last_x = odom::get_x();
	inch_t last_y = odom::get_y();

	while (true) {
		bool l2 = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2);
		bool l1 = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1);

		if (l2 && !l1) {
			shift_torque();
			automatic = false;
		}

		if (l1 && !l2) {
			shift_speed();
			automatic = false;
		}

		if (l1 && l2) {
			automatic = true;
		}

		if (!automatic) {
			pros::delay(20);
			break;
		}

		int time = pros::millis();
		inch_t x = odom::get_x();
		inch_t y = odom::get_y();

		inch_t distance = sqrt(pow<2>(y - last_y) + pow<2>(x - last_x));
		double velocity = distance.to<double>() / (time / 1000.0);

		if (velocity > shift_up && transmission.is_extended())
			shift_speed();
		else if (velocity < shift_down && !transmission.is_extended())
			shift_torque();

		prev_time = time;

		pros::delay(20);
	}
}