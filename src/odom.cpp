#include "odom.hpp"
#include "devices.hpp"
#include "units.h"

#define LEFT_OFFSET 0
#define REAR_OFFSET 0

using namespace units::math;

inch_t odom_x = 0_in;
inch_t odom_y = 0_in;
radian_t left_odom_dist = 0_rad;
radian_t rear_odom_dist = 0_rad;
degree_t imu_heading = 0_deg;

void odom::initialize() {
	odom_left.reset_position();
	odom_rear.reset_position();
	odom_left.set_reversed(true);
	odom_rear.set_reversed(true);
	imu.reset(true);

	while (std::isinf(imu.get_heading())) {
		pros::delay(200);
	}
}

// Converts robot-centric coordinates to field-centric
std::pair<inch_t, inch_t>
odom::local_to_global_coords(inch_t local_x, inch_t local_y, degree_t robot_heading) {
	degree_t heading_traveled = robot_heading;
	inch_t distance_traveled = sqrt(pow<2>(local_x) + pow<2>(local_y));

	if (local_x >= 0_in && local_y > 0_in) {
		heading_traveled =
		    degree_t(M_PI / 2) - (robot_heading + degree_t(M_PI / 2) - fabs(atan(local_y / local_x)));
	} else if (local_x > 0_in && local_y <= 0_in) {
		heading_traveled =
		    degree_t(M_PI / 2) - (robot_heading + degree_t(M_PI / 2) + fabs(atan(local_y / local_x)));
	} else if (local_x <= 0_in && local_y < 0_in) {
		heading_traveled =
		    degree_t(M_PI / 2) - (robot_heading + degree_t(3 * M_PI / 2) - fabs(atan(local_y / local_x)));
	} else if (local_x < 0_in && local_y >= 0_in) {
		heading_traveled =
		    degree_t(M_PI / 2) - (robot_heading + degree_t(3 * M_PI / 2) + fabs(atan(local_y / local_x)));
	}

	while (heading_traveled >= 2_deg * M_PI) {
		heading_traveled -= 2_deg * M_PI;
	}

	while (heading_traveled < 0_deg) {
		heading_traveled += 2_deg * M_PI;
	}

	inch_t global_x = distance_traveled * cos(heading_traveled);
	inch_t global_y = distance_traveled * sin(heading_traveled);

	return {global_x, global_y};
}

// Converts field-centric coordinates to robot-centric
std::pair<inch_t, inch_t> odom::global_to_local_coords(
    inch_t global_x, inch_t global_y, inch_t robot_x, inch_t robot_y, degree_t robot_heading
) {
	inch_t global_x_dist = global_x - robot_x;
	inch_t global_y_dist = global_y - robot_y;
	inch_t straight_dist = sqrt(pow<2>(global_x_dist) + pow<2>(global_y_dist));
	degree_t angle_to_target = 0_deg;

	if (global_x_dist >= 0_in && global_y_dist > 0_in) { // Quadrant 1
		angle_to_target = fabs(atan(global_y_dist / global_x_dist)) + robot_heading;
	} else if (global_x_dist > 0_in && global_y_dist <= 0_in) { // Quadrant 4
		angle_to_target = degree_t(2 * M_PI) - fabs(atan(global_y_dist / global_x_dist)) + robot_heading;
	} else if (global_x_dist <= 0_in && global_y_dist < 0_in) { // Quadrant 3
		angle_to_target = degree_t(M_PI) + fabs(atan(global_y_dist / global_x_dist)) - robot_heading;
	} else if (global_x_dist < 0_in && global_y_dist >= 0_in) { // Quadrant 2
		angle_to_target = degree_t(M_PI) - fabs(atan(global_y_dist / global_x_dist)) + robot_heading;
	}

	inch_t local_x = straight_dist * cos(angle_to_target);
	inch_t local_y = straight_dist * sin(angle_to_target);

	return {local_x, local_y};
}

[[noreturn]] void odom::track_position() {
	degree_t previous_heading = degree_t(imu.get_heading());
	degree_t theta;
	inch_t tracking_x = odom_x;
	inch_t tracking_y = odom_y;
	inch_t local_dist_x;
	inch_t local_dist_y;

	while (true) {
		left_odom_dist = radian_t((odom_left.get_position() / 100.0) / 360.0 *
		                 (2.75 * M_PI)); // distance traveled by left tracking wheel since last poll
		rear_odom_dist = radian_t((odom_rear.get_position() / 100.0) / 360.0 * (2.75 * M_PI));

		imu_heading = degree_t(imu.get_heading());
		theta = imu_heading - previous_heading;

		local_dist_x = 0_in;
		local_dist_y = 0_in;

		if (theta != 0_deg) {
			local_dist_x = 2 * (rear_odom_dist / theta + REAR_OFFSET) * (sin(theta / 2));
			local_dist_y = 2 * (left_odom_dist / theta + LEFT_OFFSET) * (sin(theta / 2));
		} else {
			local_dist_x = rear_odom_dist;
			local_dist_y = left_odom_dist;
		}

		std::pair<inch_t, inch_t> global =
		    local_to_global_coords(local_dist_x, local_dist_y, imu_heading);
		tracking_x += global.first;
		tracking_y += global.second;

		std::pair<inch_t, inch_t> offset =
		    local_to_global_coords(inch_t(LEFT_OFFSET), inch_t(REAR_OFFSET), imu_heading);
		odom_x = tracking_x + offset.first;
		odom_y = tracking_y + offset.second;

		odom_left.reset_position();
		odom_rear.reset_position();
		previous_heading = imu_heading;

		pros::delay(10);
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
