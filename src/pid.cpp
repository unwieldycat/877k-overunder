#include "pid.hpp"

float PIDController::calculate(float set_point, float current_pos) {
	float error = current_pos - set_point;
	error_change = error - error_prev;
	error_total += error;
	error_prev = error;

	return (error * kP + error_total * kI + error_change * kD);
}

bool PIDController::condition_met(float range) {
	if (fabs(error_prev) < range) {
		return true;
	}

	return false;
}

void PIDController::reset() {
	error_change = 0;
	error_prev = 0;
	error_total = 0;
}
