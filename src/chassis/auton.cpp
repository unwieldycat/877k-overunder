#include "main.h"
#include "pros/motors.h"
#include "units.h"
#include <cmath>

using namespace units::math;

PIDController<inch_t> drive_pid(5, 2, 5, 1000, 0.2);
PIDController<degree_t> turn_pid(2.5, 0.2, 0.5, 500, 0.5);

// ============================ Auton Functions ============================ //

void chassis::drive(int power, millisecond_t time) {
	chassis::drive(power, (degree_t)imu.get_rotation(), time);
}

void chassis::drive(int power, degree_t heading, millisecond_t time) {
	millisecond_t start_time = (millisecond_t)pros::millis();
	millisecond_t current_time;
	double turn;

	turn_pid.reset();
	drive_left.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
	drive_right.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);

	do {
		current_time = (millisecond_t)pros::millis();
		turn = turn_pid.calculate(heading, degree_t(imu.get_rotation()));
		drive_left.move(power + turn);
		drive_right.move(power - turn);
		pros::delay(20);
	} while (current_time < time + start_time);

	drive_left.brake();
	drive_right.brake();
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

	drive_left.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
	drive_right.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);

	do {
		traveled = sqrt(pow<2>(odom::get_x() - origin_x) + pow<2>(odom::get_y() - origin_y));
		current_rot = (degree_t)imu.get_rotation();
		drive = drive_pid.calculate(distance, traveled);
		turn = turn_pid.calculate(heading, current_rot);
		drive_left.move(drive + turn);
		drive_right.move(drive - turn);
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

	heading += (degree_t)imu.get_rotation();
	turn_pid.reset();

	drive_left.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
	drive_right.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);

	do {
		current_hdg = degree_t(imu.get_rotation());
		output = turn_pid.calculate(heading, current_hdg);
		drive_left.move(output);
		drive_right.move(-output);
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

	drive_left.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
	drive_right.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);

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
