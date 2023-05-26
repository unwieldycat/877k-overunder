#include "chassis.hpp"
#include "devices.hpp"
#include "odometry.hpp"
#include "pid.hpp"

PIDController drive_pid(1, 1, 1);

void chassis::drive(int distance) {
	drive_pid.reset();
	drive_pid.set_goal(distance, 0.5);

	float current_pos;
	float output;

	while (!drive_pid.goal_met()) {
		current_pos = odom::get_x();
		output = drive_pid.calculate(current_pos);
		drive_left.move(output);
		drive_right.move(-output);
	}

	drive_left.brake();
	drive_right.brake();
}