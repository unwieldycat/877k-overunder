#include "pid.hpp"

float PIDController::calculate(float set_point, float current_pos) {
	error = current_pos - set_point;
	error_change = error - error_prev;
	error_total += error;
	error_prev = error;

	return (error * kP + error_total * kI + error_change * kD);
}

void PIDController::set_gains(float kP, float kI, float kD) {
	this->kP = kP;
	this->kI = kI;
	this->kD = kD;
}

float PIDController::get_error() { return error; }

void PIDController::reset() {
	error_change = 0;
	error_prev = 0;
	error_total = 0;
}
