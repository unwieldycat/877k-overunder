#include "chassis.hpp"
#include "devices.hpp"
#include "odom.hpp"
#include "pid.hpp"

#define kP 0.5
#define kI 0.5
#define kD 0.5

#define WHEEL_OFFSET 7.6

// ============================ Auton Functions ============================ //

void chassis::drive(foot_t distance) {
	PIDController<foot_t> drive_pid(kP, kI, kD);
	PIDController<foot_t> turn_pid(kP, kI, kD);

	double drive;
	double turn;

	while (drive_pid.get_error() != 0) {
		drive = drive_pid.calculate(distance, foot_t(odom::get_x()));
		turn = turn_pid.calculate(0_ft, foot_t(odom::get_y()));
		drive_left.move(drive - turn);
		drive_right.move(-(drive + turn));
		pros::delay(20);
	}

	drive_left.brake();
	drive_right.brake();
}

void chassis::turn_abs(degree_t heading) {
	PIDController<degree_t> pid(kP, kI, kD);

	double output;
	double dir = 1;
	degree_t current_hdg = degree_t(imu.get_heading());

	if ((heading - current_hdg) > 180_deg) dir = -1;

	while (pid.get_error() != 0) {
		output = pid.calculate(heading, degree_t(imu.get_heading()));
		drive_left.move(-output * dir);
		drive_right.move(output * dir);
		pros::delay(20);
	}

	drive_left.brake();
	drive_right.brake();
}

void chassis::turn_rel(degree_t degrees) {
	degree_t heading = degree_t(imu.get_heading()) + degrees;
	if (heading > 180_deg) heading -= 360_deg;
	if (heading < 0_deg) heading = units::math::fabs(heading);

	turn_abs(heading);
}