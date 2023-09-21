#include "alt_odom.hpp"
#include "devices.hpp"
#include "odom.hpp"
#include "units.h"

#define DIAMETRE 2.75_in

using namespace units::math;

inch_t current_x;
inch_t current_y;

void alt_odom::track_pos() {
	degree_t robot_heading = (degree_t)(imu.get_heading()), true_heading;
	inch_t left_dist, rear_dist, total_dist;

	while (true) {
		left_dist = (radian_t)((degree_t)(odom_left.get_position() / 100.0)) * (DIAMETRE / 2);
		rear_dist = (radian_t)((degree_t)(odom_rear.get_position() / 100.0)) * (DIAMETRE / 2);

		total_dist = sqrt(pow<2>(left_dist) + pow<2>(rear_dist));

		true_heading = robot_heading + atan2(left_dist, rear_dist);

		if (true_heading >= 360_deg)
			true_heading -= 360_deg;
		else if (true_heading < 0_deg)
			true_heading += 360_deg;

		current_x += (total_dist * cos(true_heading));
		current_y += (total_dist * sin(true_heading));

		odom_left.reset_position();
		odom_rear.reset_position();

		pros::delay(10);
	}
}

inch_t alt_odom::x_coord() { return current_x; }

inch_t alt_odom::y_coord() { return current_y; }