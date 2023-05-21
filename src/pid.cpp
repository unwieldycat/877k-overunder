#include "pid.hpp"

float PIDController::calculate(float current_pos) {
	float error = current_pos - set_point;
	error_change = error - error_prev;
	error_total += error;
	error_prev = error;

	return (error * kP + error_total * kI + error_change * kD);
}

bool PIDController::goal_met() {
	if (fabs(error_prev) < range) {
		return true;
	}

	return false;
}

void PIDController::set_goal(float set_point, float range) {
	this->set_point = set_point;
	this->range = range;
}

void PIDController::reset() {
	error_change = 0;
	error_prev = 0;
	error_total = 0;
}
