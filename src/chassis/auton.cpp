#include "main.h"
#include <cmath>

using namespace units::math;

// ============================ Auton Functions ============================ //

void chassis::drive(int power) {
	drive_left.move(power);
	drive_right.move(power);
}

void chassis::drive(foot_t distance) {
	PIDController<inch_t> drive_pid(2, 0, 0);
	PIDController<degree_t> turn_pid(1, 0, 0);

	inch_t origin_x = odom::get_x();
	inch_t origin_y = odom::get_y();
	inch_t traveled;
	double drive;
	double turn;
	degree_t target_rot = (degree_t)imu.get_rotation();
	degree_t current_rot;

	do {
		traveled = sqrt(pow<2>(odom::get_x() - origin_x) + pow<2>(odom::get_y() - origin_y));
		current_rot = (degree_t)imu.get_rotation();
		drive = drive_pid.calculate(distance, traveled);
		turn = turn_pid.calculate(target_rot, current_rot);
		drive_left.move(drive - turn);
		drive_right.move(drive + turn);
		pros::delay(20);
	} while (!drive_pid.settled());

	drive_left.brake();
	drive_right.brake();
}

void chassis::turn_abs(degree_t heading) {
	PIDController<degree_t> pid(3, 0.1, 1);

	double output;
	degree_t current_hdg;

	heading += (degree_t)360.0 * ((int)imu.get_rotation() / 360);

	do {
		current_hdg = degree_t(imu.get_rotation());
		output = pid.calculate(heading, current_hdg);
		drive_left.move(output);
		drive_right.move(-output);
		pros::delay(20);
	} while (!pid.settled());

	drive_left.brake();
	drive_right.brake();
}

void chassis::turn_rel(degree_t degrees) {
	PIDController<degree_t> pid(3, 0.1, 1);

	double output;
	degree_t current_rot;
	degree_t end_rot = (degree_t)imu.get_rotation() + degrees;

	do {
		current_rot = degree_t(imu.get_rotation());
		output = pid.calculate(end_rot, current_rot);
		drive_left.move(output);
		drive_right.move(-output);
		pros::delay(20);
	} while (!pid.settled());

	drive_left.brake();
	drive_right.brake();
}
