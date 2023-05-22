#include "odometry.hpp"
#include "devices.hpp"

//Disregard for now, massive mess - will clean up
class Odometry {
private: 
	float odom_x = 0;
	float odom_y = 0;
	const float LEFT_OFFSET = 0;
	const float RIGHT_OFFSET = 0;
	const float REAR_OFFSET = 0;
	float left_odom_dist = 0;
	float right_odom_dist = 0;
	float rear_odom_dist = 0;
	float imu_heading = 0;

public:
	Odometry() {
		odom_left.reset_position();
		odom_right.reset_position();
		odom_rear.reset_position();
		odom_left.set_reversed(true);
		odom_right.set_reversed(false);
		odom_rear.set_reversed(true);
		imu.reset();
	}

	//Converts robot-centric coordinates to field-centric
	float localToGlobalCoords(float local_x, float local_y, float robot_heading, bool return_x = true) {
		float heading_traveled = robot_heading;
		float distance_traveled = sqrt(pow(local_x, 2) + pow(local_y, 2));

		if (local_x >= 0 && local_y > 0) {
			heading_traveled = (M_PI/2) - (robot_heading + (M_PI/2) - fabs(atan(local_y / local_x)));
		} else if (local_x > 0 && local_y <= 0) {
			heading_traveled = (M_PI/2) - (robot_heading + (M_PI/2) + fabs(atan(local_y / local_x)));
		} else if (local_x <= 0 && local_y < 0) {
			heading_traveled = (M_PI/2) - (robot_heading + (3*M_PI/2) - fabs(atan(local_y / local_x)));
		} else if (local_x < 0 && local_y >= 0) {
			heading_traveled = (M_PI/2) - (robot_heading + (3*M_PI/2) + fabs(atan(local_y / local_x)));
		} 
		while (heading_traveled >= 2*M_PI) {
			heading_traveled -= 2*M_PI;
		} 
		while (heading_traveled < 0) {
			heading_traveled += 2*M_PI;
		}

		float global_x = distance_traveled * cos(heading_traveled);
		float global_y = distance_traveled * sin(heading_traveled);

		if (return_x) {
			return global_x;
		} else {
			return global_y;
		}
	}

	//Converts field-centric coordinates to robot-centric
	float globalToLocalCoords(float global_x, float global_y, float robot_x, float robot_y, float robot_heading, bool return_x = true) {
		float global_x_dist = global_x - robot_x;
		float global_y_dist = global_y - robot_y;
		float straight_dist = sqrt(pow(global_x_dist, 2) + pow(global_y_dist, 2));
		float angle_to_target = 0;

		if (global_x_dist >= 0 && global_y_dist > 0) {          //Quadrant 1
			angle_to_target = fabs(atan(global_y_dist / global_x_dist)) + robot_heading;
		} else if (global_x_dist > 0 && global_y_dist <= 0) {   //Quadrant 4
			angle_to_target = 2*M_PI - fabs(atan(global_y_dist / global_x_dist)) + robot_heading;
		} else if (global_x_dist <= 0 && global_y_dist < 0) {   //Quadrant 3
			angle_to_target = M_PI + fabs(atan(global_y_dist / global_x_dist)) - robot_heading;
		} else if (global_x_dist < 0 && global_y_dist >= 0) {   //Quadrant 2
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

	void track_position() {
		float previous_heading = 0;
		float theta = 0;
		float tracking_x = 0;
		float tracking_y = 0;
		float localXDist = 0;
		float localYDist = 0;

		while (true) {
			left_odom_dist = odom_left.get_position()*100/360 * (2.75 * M_PI);   //distance traveled by left tracking wheel since last poll
			right_odom_dist = odom_left.get_position()*100/360 * (2.75 * M_PI);
			rear_odom_dist = odom_left.get_position()*100/360 * (2.75 * M_PI);

			imu_heading = imu.get_heading();
			theta = imu_heading - previous_heading;

			localXDist = 0;
			localYDist = 0;
			if (theta != 0) {
			localXDist = 2 * (rear_odom_dist / theta + REAR_OFFSET) * (sin(theta / 2));
			localYDist = (left_odom_dist + right_odom_dist) / theta * (sin(theta / 2));
			} else {
			localXDist = rear_odom_dist;
			localYDist = (right_odom_dist + left_odom_dist) / 2;
			}

			tracking_x += localToGlobalCoords(localXDist, localYDist, imu_heading, true);
			tracking_y += localToGlobalCoords(localXDist, localYDist, imu_heading, false);

			odom_x = tracking_x + localToGlobalCoords(RIGHT_OFFSET - LEFT_OFFSET, REAR_OFFSET, imu_heading, true);
			odom_y = tracking_y + localToGlobalCoords(RIGHT_OFFSET - LEFT_OFFSET, REAR_OFFSET, imu_heading, false);

			odom_left.reset_position();
			odom_right.reset_position();
			odom_rear.reset_position();
			previous_heading = imu_heading;

			pros::delay(100);
		}
	}

	float get_x() {
		return odom_x;
	}
	float get_y() {
		return odom_y;
	}
};