#include "chassis.hpp"
#include "devices.hpp"
#include "odom.hpp"
#include "pid.hpp"

#define kP 0.5
#define kI 0.5
#define kD 0.5

void chassis::drive(float distance) {
	PIDController drive_pid(kP, kI, kD);
	PIDController turn_pid(kP, kI, kD);

	float drive;
	float turn;

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

void chassis::turn_abs(float heading) {
	PIDController pid(kP, kI, kD);
	float output;

	while (pid.get_error() != 0) {
		output = pid.calculate(heading, imu.get_heading());
		drive_left.move(-output);
		drive_right.move(output);
		pros::delay(20);
	}

	drive_left.brake();
	drive_right.brake();
}

void chassis::turn_rel(float degrees) {
	PIDController pid(kP, kI, kD);
	double output;
	double dir;

	double heading = imu.get_heading() + degrees;
	if (heading > 180) heading -= 360;
	if (heading < 0) {
		dir = -1;
		heading = fabs(heading);
	}

	while (pid.get_error() != 0) {
		output = pid.calculate(heading, imu.get_heading());
		drive_left.move(-output * dir);
		drive_right.move(output * dir);
		pros::delay(20);
	}

	drive_left.brake();
	drive_right.brake();
}