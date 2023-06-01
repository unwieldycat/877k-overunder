#include "chassis.hpp"
#include "devices.hpp"
#include "odom.hpp"
#include "pid.hpp"

#define kP 0.5
#define kI 0.5
#define kD 0.5

#define WHEEL_OFFSET 7.6

// ============================ Auton Functions ============================ //

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

// ========================= User Control Functions ========================= //

chassis::ControlMode drive_mode;

void tank_drive() {
	int left_power = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
	int right_power = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
	drive_left.move(left_power);
	drive_right.move(right_power);
}

void arcade_drive() {
	int left_y = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
	int right_x = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
	drive_left.move(left_y + right_x);
	drive_left.move(left_y + right_x);
}

void curvature_drive() {
	int power = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
	float curvature =
	    controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X) / 127.0 * WHEEL_OFFSET;
	if (curvature != 0.0) {
		float radius = 1.0 / curvature;
		drive_left.move(power * (radius + WHEEL_OFFSET) / radius);
		drive_left.move(power * (radius - WHEEL_OFFSET) / radius);
	} else {
		drive_left.move(power);
		drive_right.move(power);
	}
}

void chassis::user_control() {
	while (true) {
		switch (drive_mode) {
		case ControlMode::TANK:
			tank_drive();
			break;
		case ControlMode::ARCADE:
			arcade_drive();
			break;
		case ControlMode::CURVE:
			curvature_drive();
			break;
		}

		pros::delay(50);
	}
}

void chassis::set_mode(chassis::ControlMode new_mode) { drive_mode = new_mode; }

chassis::ControlMode chassis::get_mode() { return drive_mode; }