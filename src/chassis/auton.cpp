#include "auton.hpp"
#include "devices.hpp"
#include "odom.hpp"
#include "pid.hpp"
#include "units.h"

#define WHEEL_OFFSET 7.6

using namespace units::math;

// ============================ Auton Functions ============================ //

void chassis::drive(int power) {
	drive_left.move(power);
	drive_right.move(power);
}

void chassis::drive(foot_t distance) {
	PIDController<inch_t> drive_pid(2, 0, 0);
	PIDController<degree_t> turn_pid(1, 0, 0);

	double drive;
	double turn;
	degree_t target_hdg = (degree_t)imu.get_heading();
	degree_t current_hdg;

	do {
		current_hdg = (degree_t)imu.get_heading();
		drive = drive_pid.calculate(distance, odom::get_y());
		turn = turn_pid.calculate(target_hdg, current_hdg);
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
	degree_t heading = degree_t(imu.get_heading()) + degrees;
	if (heading > 180_deg) heading -= 360_deg;
	if (heading < 0_deg) heading = units::math::fabs(heading);

	turn_abs(heading);
}
