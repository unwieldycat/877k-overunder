#include "odometry.hpp"
#include "devices.hpp"

Odometry::Odometry() {
	odom_left.reset_position();
	odom_right.reset_position();
	odom_rear.reset_position();
	odom_left.set_reversed(true);
	odom_right.set_reversed(false);
	odom_rear.set_reversed(true);
	imu.reset();
}

// Converts robot-centric coordinates to field-centric
float Odometry::local_to_global_coords(
    float local_x, float local_y, float robot_heading, bool return_x = true
) {
	float heading_traveled = robot_heading;
	float distance_traveled = sqrt(pow(local_x, 2) + pow(local_y, 2));

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

	float global_x = distance_traveled * cos(heading_traveled);
	float global_y = distance_traveled * sin(heading_traveled);

	if (return_x) {
		return global_x;
	} else {
		return global_y;
	}
}

// Converts field-centric coordinates to robot-centric
float Odometry::global_to_local_coords(
    float global_x, float global_y, float robot_x, float robot_y, float robot_heading,
    bool return_x = true
) {
	float global_x_dist = global_x - robot_x;
	float global_y_dist = global_y - robot_y;
	float straight_dist = sqrt(pow(global_x_dist, 2) + pow(global_y_dist, 2));
	float angle_to_target = 0;

	if (global_x_dist >= 0 && global_y_dist > 0) { // Quadrant 1
		angle_to_target = fabs(atan(global_y_dist / global_x_dist)) + robot_heading;
	} else if (global_x_dist > 0 && global_y_dist <= 0) { // Quadrant 4
		angle_to_target = 2 * M_PI - fabs(atan(global_y_dist / global_x_dist)) + robot_heading;
	} else if (global_x_dist <= 0 && global_y_dist < 0) { // Quadrant 3
		angle_to_target = M_PI + fabs(atan(global_y_dist / global_x_dist)) - robot_heading;
	} else if (global_x_dist < 0 && global_y_dist >= 0) { // Quadrant 2
		angle_to_target = M_PI - fabs(atan(global_y_dist / global_x_dist)) + robot_heading;
	}

	float local_x = straight_dist * cos(angle_to_target);
	float local_y = straight_dist * sin(angle_to_target);

	if (return_x) {
		return local_x;
	} else {
		return local_y;
	}
}

void Odometry::track_position() {
	float previous_heading = 0;
	float theta = 0;
	float tracking_x = 0;
	float tracking_y = 0;
	float local_dist_x = 0;
	float local_dist_y = 0;

	while (true) {
		left_odom_dist = odom_left.get_position() * 100 / 360.0 *
		                 (2.75 * M_PI); // distance traveled by left tracking wheel since last poll
		right_odom_dist = odom_left.get_position() * 100 / 360.0 * (2.75 * M_PI);
		rear_odom_dist = odom_left.get_position() * 100 / 360.0 * (2.75 * M_PI);

		imu_heading = imu.get_heading();
		theta = imu_heading - previous_heading;

		local_dist_x = 0;
		local_dist_y = 0;
		if (theta != 0) {
			local_dist_x = 2 * (rear_odom_dist / theta + REAR_OFFSET) * (sin(theta / 2));
			local_dist_y = (left_odom_dist + right_odom_dist) / theta * (sin(theta / 2));
		} else {
			local_dist_x = rear_odom_dist;
			local_dist_y = (right_odom_dist + left_odom_dist) / 2;
		}

		tracking_x += local_to_global_coords(local_dist_x, local_dist_y, imu_heading, true);
		tracking_y += local_to_global_coords(local_dist_x, local_dist_y, imu_heading, false);

		odom_x = tracking_x +
		         local_to_global_coords(RIGHT_OFFSET - LEFT_OFFSET, REAR_OFFSET, imu_heading, true);
		odom_y =
		    tracking_y +
		    local_to_global_coords(RIGHT_OFFSET - LEFT_OFFSET, REAR_OFFSET, imu_heading, false);

		odom_left.reset_position();
		odom_right.reset_position();
		odom_rear.reset_position();
		previous_heading = imu_heading;

		pros::delay(100);
	}
}

void Odometry::calibrate(float robot_x = 0, float robot_y = 0, float heading = 0) {
	odom_x = robot_x;
	odom_y = robot_y;
	imu.reset();
	imu.set_heading(heading);
}

float Odometry::get_x() { return odom_x; }
float Odometry::get_y() { return odom_y; }
