#include "chassis.hpp"
#include "devices.hpp"
#include "odom.hpp"
#include "pid.hpp"

#define kP 0.5
#define kI 0.5
#define kD 0.5

void chassis::drive(double distance) {
	PIDController drive_pid(kP, kI, kD);
	PIDController turn_pid(kP, kI, kD);

	double drive;
	double turn;

	while (drive_pid.get_error() != 0) {
		drive = drive_pid.calculate(distance, odom::get_x());
		turn = turn_pid.calculate(0, odom::get_y());
		drive_left.move(drive - turn);
		drive_right.move(-(drive + turn));
		pros::delay(20);
	}

	drive_left.brake();
	drive_right.brake();
}

void chassis::turn_abs(double heading) {
	PIDController pid(kP, kI, kD);
	double output;
	double dir = 1;
	double current_hdg = imu.get_heading();

	if ((heading - current_hdg) > 180) dir = -1;

	while (pid.get_error() != 0) {
		output = pid.calculate(heading, imu.get_heading());
		drive_left.move(-output * dir);
		drive_right.move(output * dir);
		pros::delay(20);
	}

	drive_left.brake();
	drive_right.brake();
}

void chassis::turn_rel(double degrees) {
	double heading = imu.get_heading() + degrees;
	if (heading > 180) heading -= 360;
	if (heading < 0) heading = fabs(heading);

	turn_abs(heading);
}