#include "odom.hpp"
#include "devices.hpp"
#include <cmath>

#define LEFT_OFFSET 0
#define REAR_OFFSET 0

double odom_x = 0;
double odom_y = 0;
double left_odom_dist = 0;
double rear_odom_dist = 0;
double imu_heading = 0;

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
std::pair<double, double> odom::local_to_global_coords(
    double local_x, double local_y, double robot_heading
) {
	double heading_traveled = robot_heading;
	double distance_traveled = sqrt(pow(local_x, 2) + pow(local_y, 2));

	if (local_x >= 0 && local_y > 0) {
		heading_traveled =
		    (M_PI / 2) - (robot_heading + (M_PI / 2) - fabs(atan(local_y / local_x)));
	} else if (local_x > 0 && local_y <= 0) {
		heading_traveled =
		    (M_PI / 2) - (robot_heading + (M_PI / 2) + fabs(atan(local_y / local_x)));
	} else if (local_x <= 0 && local_y < 0) {
		heading_traveled =
		    (M_PI / 2) - (robot_heading + (3 * M_PI / 2) - fabs(atan(local_y / local_x)));
	} else if (local_x < 0 && local_y >= 0) {
		heading_traveled =
		    (M_PI / 2) - (robot_heading + (3 * M_PI / 2) + fabs(atan(local_y / local_x)));
	}

	while (heading_traveled >= 2 * M_PI) {
		heading_traveled -= 2 * M_PI;
	}

	while (heading_traveled < 0) {
		heading_traveled += 2 * M_PI;
	}

	double global_x = distance_traveled * cos(heading_traveled);
	double global_y = distance_traveled * sin(heading_traveled);

	return { global_x, global_y };
}

// Converts field-centric coordinates to robot-centric
std::pair <double, double> odom::global_to_local_coords(
    double global_x, double global_y, double robot_x, double robot_y, double robot_heading
) {
	double global_x_dist = global_x - robot_x;
	double global_y_dist = global_y - robot_y;
	double straight_dist = sqrt(pow(global_x_dist, 2) + pow(global_y_dist, 2));
	double angle_to_target = 0;

	if (global_x_dist >= 0 && global_y_dist > 0) { // Quadrant 1
		angle_to_target = fabs(atan(global_y_dist / global_x_dist)) + robot_heading;
	} else if (global_x_dist > 0 && global_y_dist <= 0) { // Quadrant 4
		angle_to_target = 2 * M_PI - fabs(atan(global_y_dist / global_x_dist)) + robot_heading;
	} else if (global_x_dist <= 0 && global_y_dist < 0) { // Quadrant 3
		angle_to_target = M_PI + fabs(atan(global_y_dist / global_x_dist)) - robot_heading;
	} else if (global_x_dist < 0 && global_y_dist >= 0) { // Quadrant 2
		angle_to_target = M_PI - fabs(atan(global_y_dist / global_x_dist)) + robot_heading;
	}

	double local_x = straight_dist * cos(angle_to_target);
	double local_y = straight_dist * sin(angle_to_target);

	return { local_x, local_y };
}

// FIXME: Odom output is wrong
// FIXME: X and Y accumulate after every loop
void odom::track_position() {
	double previous_heading = 0;
	double theta = 0;
	double tracking_x = 0;
	double tracking_y = 0;
	double local_dist_x = 0;
	double local_dist_y = 0;

	while (true) {
		left_odom_dist = odom_left.get_position() * 100 / 360.0 *
		                 (2.75 * M_PI); // distance traveled by left tracking wheel since last poll
		rear_odom_dist = odom_rear.get_position() * 100 / 360.0 * (2.75 * M_PI);

		imu_heading = imu.get_heading();
		theta = imu_heading - previous_heading;

		local_dist_x = 0;
		local_dist_y = 0;
		if (theta != 0) {
			local_dist_x = 2 * (rear_odom_dist / theta + REAR_OFFSET) * (sin(theta / 2));
			local_dist_y = 2 * (left_odom_dist / theta + LEFT_OFFSET) * (sin(theta / 2));
		} else {
			local_dist_x = rear_odom_dist;
			local_dist_y = left_odom_dist;
		}
		
		std::pair<double, double> global = local_to_global_coords(local_dist_x, local_dist_y, imu_heading);
		tracking_x += global.first;
		tracking_y += global.second;

		std::pair<double, double> offset = local_to_global_coords(LEFT_OFFSET, REAR_OFFSET, imu_heading);
		odom_x = tracking_x + offset.first;
		odom_y = tracking_y + offset.second;

		odom_left.reset_position();
		odom_rear.reset_position();
		previous_heading = imu_heading;

		pros::delay(10);
	}
}

void odom::calibrate(double robot_x = 0, double robot_y = 0, double heading = 0) {
	odom_x = robot_x;
	odom_y = robot_y;
	imu.reset();
	imu.set_heading(heading);
}

double odom::get_x() { return odom_x; }
double odom::get_y() { return odom_y; }
