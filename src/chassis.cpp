#include "chassis.hpp"
#include "devices.hpp"
#include "odometry.hpp"
#include "pid.hpp"

PIDController drive_pid(1, 1, 1);
PIDController turn_pid(1, 1, 1);

void chassis::drive(int distance) {
	drive_pid.set_goal(distance, 0.5);
	turn_pid.set_goal(0, 0.5);

	float drive;
	float turn;

	while (!drive_pid.goal_met()) {
		drive = drive_pid.calculate(odom::get_x());
		turn = turn_pid.calculate(odom::get_y());
		drive_left.move(drive - turn);
		drive_right.move(-(drive + turn));
	}

	drive_left.brake();
	drive_right.brake();
}