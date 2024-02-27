#include "main.h"

using namespace units::math;

PIDController<inch_t> drive_pid(4, 0.2, 0.6, 1000, 2);
PIDController<degree_t> turn_pid(3, 0.3, 0.3, 750, 2);
PIDController<degree_t> align_pid(5, 0, 0, 1000, 0.5);

// ============================ Helper Functions ============================ //

degree_t optimize_turn(degree_t heading) {
	// + or - 360 to heading to make turn as short as possible
	while (heading - degree_t(imu.get_rotation()) >= 180_deg)
		heading -= 360_deg;
	while (heading - degree_t(imu.get_rotation()) <= -180_deg)
		heading += 360_deg;

	return heading;
}

// ============================ Drive Functions ============================ //

void chassis::drive(int power, millisecond_t time) {
	millisecond_t start_time = (millisecond_t)pros::millis();
	millisecond_t current_time;

	drive_left.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
	drive_right.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);

	do {
		current_time = (millisecond_t)pros::millis();
		drive_left.move(power);
		drive_right.move(power);
		pros::delay(20);
	} while (current_time < time + start_time);

	drive_left.brake();
	drive_right.brake();
}

void chassis::drive(int power, degree_t heading, millisecond_t time) {
	millisecond_t start_time = (millisecond_t)pros::millis();
	millisecond_t current_time;
	double turn;

	heading = optimize_turn(heading);

	align_pid.reset();
	drive_left.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
	drive_right.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);

	do {
		current_time = (millisecond_t)pros::millis();
		turn = align_pid.calculate(heading, degree_t(imu.get_rotation()));
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

	double sign = 1;

	if (distance < 0_ft) {
		distance = abs(distance);
		sign = -1;
	}

	heading = optimize_turn(heading);

	drive_pid.reset();
	align_pid.reset();

	drive_left.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
	drive_right.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);

	do {
		traveled = sqrt(pow<2>(odom::get_x() - origin_x) + pow<2>(odom::get_y() - origin_y));
		current_rot = (degree_t)imu.get_rotation();
		drive = drive_pid.calculate(distance, traveled);
		turn = align_pid.calculate(heading, current_rot);
		drive_left.move(sign * drive + turn);
		drive_right.move(sign * drive - turn);
		pros::delay(20);
	} while (!drive_pid.settled());

	drive_left.brake();
	drive_right.brake();
}

// ============================= Turn Functions ============================= //

void do_turn(double output, chassis::TurnSide side) {
	switch (side) {
	case chassis::TurnSide::Left:
		drive_left.move(output);
		drive_right.move(0);
		break;
	case chassis::TurnSide::Right:
		drive_left.move(0);
		drive_right.move(-output);
		break;
	default:
		drive_left.move(output);
		drive_right.move(-output);
	}
}

void chassis::turn_abs(degree_t heading, TurnSide side) {
	double output;
	degree_t current_hdg;

	heading = optimize_turn(heading);

	turn_pid.reset();
	drive_left.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
	drive_right.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);

	do {
		current_hdg = degree_t(imu.get_rotation());
		output = turn_pid.calculate(heading, current_hdg);
		do_turn(output, side);
		pros::delay(20);
	} while (!turn_pid.settled());

	drive_left.brake();
	drive_right.brake();
}

void chassis::turn_rel(degree_t degrees, TurnSide side) {
	double output;
	degree_t current_rot;
	degree_t end_rot = (degree_t)imu.get_rotation() + degrees;
	turn_pid.reset();

	drive_left.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
	drive_right.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);

	do {
		current_rot = degree_t(imu.get_rotation());
		output = turn_pid.calculate(end_rot, current_rot);
		do_turn(output, side);
		pros::delay(20);
	} while (!turn_pid.settled());

	drive_left.brake();
	drive_right.brake();
}

// ========================== Temporary Functions ========================== //

void chassis::d_drive(double dist, int power) {
	std::cout << drive_left.get_position() << "\n";
	double target_pos = dist / (4 * M_PI) * 360 + drive_left.get_position();
	std::cout << target_pos << "\n";

	if (target_pos > drive_left.get_position()) {
		do {
			drive_left.move(power);
			drive_right.move(power);
			pros::delay(20);
		} while (drive_left.get_position() < target_pos);
	} else {
		do {
			drive_left.move(-power);
			drive_right.move(-power);
			pros::delay(20);
		} while (drive_left.get_position() > target_pos);
	}

	drive_left.brake();
	drive_right.brake();
}

void chassis::d_turn(double angle, int power) {
	double dist = angle / 360 * (15 * M_PI) / (4 * M_PI) * 360 + drive_left.get_position();

	if (dist > drive_left.get_position()) {
		do {
			drive_left.move(power);
			drive_right.move(-power);
			pros::delay(20);
		} while (drive_left.get_position() < dist);
	} else {
		do {
			drive_left.move(-power);
			drive_right.move(power);
			pros::delay(20);
		} while (drive_left.get_position() > dist);
	}

	drive_left.brake();
	drive_right.brake();
}
