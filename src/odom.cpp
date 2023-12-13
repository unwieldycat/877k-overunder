#include "odom.hpp"
#include "devices.hpp"
#include "units.h"

const std::pair<inch_t, inch_t> left_offset = {-4.829_in, 1.4675_in};
const std::pair<inch_t, inch_t> rear_offset = {1.1655_in, 0.825_in};

using namespace units::math;

inch_t odom_x = 0_in;
inch_t odom_y = 0_in;
inch_t left_odom_dist = 0_in;
inch_t rear_odom_dist = 0_in;
degree_t imu_heading = 0_deg;

void odom::initialize() {
	odom_left.reset_position();
	odom_rear.reset_position();
	odom_left.set_reversed(false);
	odom_rear.set_reversed(false);
	imu.reset(true);

	while (std::isinf(imu.get_heading())) {
		pros::delay(200);
	}
}

[[noreturn]] void odom::track_position() {
	auto left_dist = 0_rad * 0_in, rear_dist = 0_rad * 0_in;
	inch_t y_radius, x_radius, local_x, local_y, total_dist;
	degree_t prev_heading = (degree_t)(imu.get_heading()), heading_change, robot_heading,
	         local_true_heading, global_true_heading, left_odom, rear_odom;
	while (true) {
		left_odom = (degree_t)(odom_left.get_position() / 100.0);
		rear_odom = (degree_t)(odom_rear.get_position() / 100.0);
		robot_heading = (degree_t)(imu.get_heading());
		left_dist = (radian_t)(left_odom)*2.75_in / 2;
		rear_dist = (radian_t)(rear_odom)*2.75_in / 2;
		heading_change = robot_heading - prev_heading;

		if (heading_change != 0_deg) {
			int sign = (rear_offset.second > rear_offset.first)? 1 : -1;
			x_radius = (rear_dist / heading_change) + sign * sqrt(pow<2>(rear_offset.second) + pow<2>(rear_offset.first));
			sign = (left_offset.second > left_offset.first)? 1 : -1;
			y_radius = (left_dist / heading_change) + sign * sqrt(pow<2>(left_offset.second) + pow<2>(left_offset.first));
			local_x = 2 * x_radius * sin(heading_change / 2);
			local_y = 2 * y_radius * sin(heading_change / 2);
		} else {
			local_x = rear_dist / 1_rad;
			local_y = left_dist / 1_rad;
		}

		local_true_heading = atan2(local_y, local_x);
		if (local_x == 0_in && local_y == 0_in) local_true_heading = 0_deg;
		global_true_heading = local_true_heading + robot_heading;
		while (global_true_heading > 180_deg)
			global_true_heading -= 360_deg;
		while (global_true_heading < 180_deg)
			global_true_heading += 360_deg;

		total_dist = sqrt(pow<2>(local_x) + pow<2>(local_y));
		odom_x += total_dist * cos(global_true_heading);
		odom_y += total_dist * sin(global_true_heading);

		odom_left.reset_position();
		odom_rear.reset_position();
		left_odom = 0_deg;
		rear_odom = 0_deg;
		prev_heading = robot_heading;

		pros::delay(20);
	}
}

void odom::calibrate(inch_t robot_x = 0_in, inch_t robot_y = 0_in, degree_t heading = 0_deg) {
	odom_x = robot_x;
	odom_y = robot_y;
	imu.reset(true);
	imu.set_heading(heading.to<double>());
}

inch_t odom::get_x() { return odom_x; }
inch_t odom::get_y() { return odom_y; }
