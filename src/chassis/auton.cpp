#include "main.h"
#include <cmath>

using namespace units::math;

// TODO: Tune better
PIDController<inch_t> drive_pid(5, 2, 5);
PIDController<degree_t> turn_pid(5, 1, 0.8);

// ============================ Auton Functions ============================ //

void chassis::drive(int power) {
	drive_left.move(power);
	drive_right.move(power);
}

void chassis::drive(foot_t distance) { chassis::drive(distance, (degree_t)imu.get_rotation()); }

void chassis::drive(foot_t distance, degree_t heading) {
	inch_t origin_x = odom::get_x();
	inch_t origin_y = odom::get_y();
	inch_t traveled;
	double drive;
	double turn;
	degree_t current_rot;

	drive_pid.reset();
	turn_pid.reset();

	do {
		traveled = sqrt(pow<2>(odom::get_x() - origin_x) + pow<2>(odom::get_y() - origin_y));
		current_rot = (degree_t)imu.get_rotation();
		drive = drive_pid.calculate(distance, traveled);
		turn = turn_pid.calculate(heading, current_rot);
		drive_left.move(drive + turn);
		drive_right.move(drive - turn);
		std::cout << traveled << " " << drive << "\n";
		pros::delay(20);
	} while (!drive_pid.settled());

	drive_left.brake();
	drive_right.brake();
}

void chassis::turn_abs(degree_t heading) {
	double output;
	degree_t current_hdg;

	while (heading > 180_deg)
		heading -= 180_deg;
	while (heading < -180_deg)
		heading += 180_deg;

	heading += (degree_t)360.0 * ((int)imu.get_rotation() / 360);
	turn_pid.reset();

	do {
		current_hdg = degree_t(imu.get_rotation());
		output = turn_pid.calculate(heading, current_hdg);
		drive_left.move(output);
		drive_right.move(-output);
		std::cout << output << " " << turn_pid.get_error() << "\n";
		pros::delay(20);
	} while (!turn_pid.settled());

	drive_left.brake();
	drive_right.brake();
}

void chassis::turn_rel(degree_t degrees) {
	double output;
	degree_t current_rot;
	degree_t end_rot = (degree_t)imu.get_rotation() + degrees;
	turn_pid.reset();

	do {
		current_rot = degree_t(imu.get_rotation());
		output = turn_pid.calculate(end_rot, current_rot);
		drive_left.move(output);
		drive_right.move(-output);
		pros::delay(20);
	} while (!turn_pid.settled());

	drive_left.brake();
	drive_right.brake();
}
