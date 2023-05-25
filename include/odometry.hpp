#pragma once
#include "main.h"

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
	Odometry();
	float local_to_global_coords(float local_x, float local_y, float robot_heading, bool return_x);
	float global_to_local_coords(float global_x, float global_y, float robot_x, float robot_y, float robot_heading, bool return_x);
	void track_position();
	void calibrate(float robot_x, float robot_y, float heading);
	float get_x();
	float get_y();
};