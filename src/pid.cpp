#include "pid.hpp"

double PIDController::calculate(double set_point, double current_pos) {
	error = current_pos - set_point;
	error_change = error - error_prev;
	error_total += error;
	error_prev = error;

	return (error * kP + error_total * kI + error_change * kD);
}

void PIDController::set_gains(double kP, double kI, double kD) {
	this->kP = kP;
	this->kI = kI;
	this->kD = kD;
}

double PIDController::get_error() { return error; }

void PIDController::reset() {
	error_change = 0;
	error_prev = 0;
	error_total = 0;
}
